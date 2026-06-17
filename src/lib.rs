#![no_std]

use embedded_hal_async::i2c::I2c;
use embedded_hal_async::i2c::Operation as I2cOperation;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_async::spi::Operation as SpiOperation;

// Registradores do ADXL345
const REG_DEVID: u8 = 0x00;
const REG_OFSX: u8 = 0x1E;
#[allow(unused)]
const REG_OFSY: u8 = 0x1F;
#[allow(unused)]
const REG_OFSZ: u8 = 0x20;
const REG_BW_RATE: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_DATA_FORMAT: u8 = 0x31;
const REG_DATAX0: u8 = 0x32;
const EARTH_GRAVITY: f32 = 9.80665;

#[derive(Copy, Clone, Debug)]
pub enum Address {
   PRIMARY = 0x1D,
   SECONDARY = 0x53
}

/// Formato dos dados (G-Range)
#[derive(Copy, Clone, Debug)]
pub enum Range {
    G2 = 0x00,
    G4 = 0x01,
    G8 = 0x02,
    G16 = 0x03,
}

/// Taxa de amostragem de dados (Output Data Rate)
#[derive(Copy, Clone, Debug)]
pub enum DataRate {
    Rate100Hz = 0x0A,
    Rate200Hz = 0x0B,
    Rate400Hz = 0x0C,
}

// =========================================================================
// O DRIVER PRINCIPAL (Independente de protocolo)
// =========================================================================

pub struct Adxl345Async<XBUS> {
    bus: XBUS,
    scale_factor: f32,
}

impl<XBUS> Adxl345Async<XBUS>
where
    XBUS: AsyncBus,
{
    /// Cria uma nova instância do driver a partir de um Barramento (Bus) assíncrono
    pub fn new(bus: XBUS) -> Self {
        Self { bus , scale_factor: 0.00390625}
    }

    /// Lê o ID do dispositivo (Deve retornar 0xE5)
    pub async fn get_device_id(&mut self) -> Result<u8, XBUS::Error> {
        self.bus.read_reg(REG_DEVID).await
    }

    /// Inicializa o sensor (coloca em modo de medição)
    pub async fn setup(&mut self) -> Result<(), XBUS::Error> {
        self.bus.write_reg(REG_POWER_CTL, 0x08).await?;
        Ok(())
    }

    /// Configura a escala de leitura (G-Range)
    pub async fn set_range(&mut self, range: Range) -> Result<(), XBUS::Error> {
        match range {
            Range::G2 => self.scale_factor = 0.00390625,  // 4mg/LSB
            Range::G4 => self.scale_factor = 0.0078125,   // 8mg/LSB
            Range::G8 => self.scale_factor = 0.015625,    // 16mg/LSB
            Range::G16 => self.scale_factor = 0.03125,    // 31.25mg/LSB
        }
        self.bus.write_reg(REG_DATA_FORMAT, range as u8).await?;
        Ok(())
    }

    /// Configura a taxa de amostragem de dados (Data Rate)
    pub async fn set_data_rate(&mut self, rate: DataRate) -> Result<(), XBUS::Error> {
        self.bus.write_reg(REG_BW_RATE, rate as u8).await?;
        Ok(())
    }

    /// Configura os valores de Offset (Calibração) para os eixos X, Y e Z.
    /// 
    /// Os valores devem ser informados em escala de 15.6 mg por LSB.
    /// Exemplo: Se o eixo X está lendo +8 em repouso (modo 2g), o erro é 8. 
    /// Dividindo por 4, temos 2. O offset compensatório deve ser -2.
    pub async fn set_offsets(&mut self, x: i8, y: i8, z: i8) -> Result<(), XBUS::Error> {
        // Colocamos os 3 valores em um array de bytes (cast para u8 mantém o sinal binário do i8)
        let offsets = [x as u8, y as u8, z as u8];
        
        // Enviamos o array inteiro começando no registrador OFSX (0x1E).
        // O chip vai gravar o 'x' no 0x1E, o 'y' no 0x1F e o 'z' no 0x20 automaticamente!
        self.bus.write_multiple(REG_OFSX, &offsets).await?;
        
        Ok(())
    }
    /*
    pub async fn set_offsets(&mut self, x: i8, y: i8, z: i8) -> Result<(), XBUS::Error> {
        // Os registradores aceitam um i8 (inteiro de 8 bits com sinal, de -128 a 127)
        self.bus.write_reg(REG_OFSX, x as u8).await?;
        self.bus.write_reg(REG_OFSY, y as u8).await?;
        self.bus.write_reg(REG_OFSZ, z as u8).await?;
        Ok(())
    }
    */

    /// Lê os três eixos (X, Y, Z) de aceleração de forma assíncrona
    pub async fn get_accel_raw(&mut self) -> Result<(i16, i16, i16), XBUS::Error> {
        // Lendo byte a byte de forma isolada para testar o barramento
        let mut buf = [0u8; 6];
        self.bus.read_multiple(REG_DATAX0, &mut buf).await?;

        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    pub async fn get_accel(&mut self) -> Result<(f32, f32, f32), XBUS::Error> {
        let accel = self.get_accel_raw().await?;
        let accel_g: (f32, f32, f32) = (
            (accel.0 as f32) * EARTH_GRAVITY * self.scale_factor,
            (accel.1 as f32) * EARTH_GRAVITY * self.scale_factor,
            (accel.2 as f32) * EARTH_GRAVITY * self.scale_factor,
        );

        Ok(accel_g)
    }
}


// =========================================================================
// CAMADA DE ABSTRAÇÃO DO BARRAMENTO (A mágica para aceitar I2C ou SPI)
// =========================================================================

/// Trait interna que define as operações que qualquer barramento (I2C/SPI) deve cumprir
#[allow(async_fn_in_trait)]
pub trait AsyncBus {
    type Error;
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Self::Error>;
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::Error>;
    async fn read_multiple(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    async fn write_multiple(&mut self, reg: u8, bytes: &[u8]) -> Result<(), Self::Error>;
}

/// Implementação da abstração de barramento especificamente para I2C
pub struct I2cBus<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cBus<I2C> {
    pub fn new(i2c: I2C, addr: Option<Address>) -> Self {
        Self { 
            i2c,
            address: addr.map(|a| a as u8).unwrap_or(Address::SECONDARY as u8),
        }
    }
}

impl<I2C: I2c> AsyncBus for I2cBus<I2C> {
    type Error = I2C::Error;

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, val]).await?;
        Ok(())
    }

    async fn read_multiple(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        let cmd = [reg];
        
        let mut operations = [
            I2cOperation::Write(&cmd),
            I2cOperation::Read(buf),
        ];
        
        self.i2c.transaction(self.address, &mut operations).await
    }

    async fn write_multiple(&mut self, reg: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let cmd = [reg];
        
        let mut operations = [
            I2cOperation::Write(&cmd),
            I2cOperation::Write(bytes),
        ];
        
        self.i2c.transaction(self.address, &mut operations).await
    }
}

/// Implementação da abstração de barramento especificamente para SPI
pub struct SpiBus<SPI> {
    spi: SPI,
}

impl<SPI> SpiBus<SPI> {
    /// Cria um novo barramento SPI. 
    /// O 'SPI' aqui deve ser um tipo que implemente `SpiDevice` (que já controla o pino CS)
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }
}

impl<SPI: SpiDevice> AsyncBus for SpiBus<SPI> {
    type Error = SPI::Error;

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Self::Error> {
        // Bit 7 (MSB) em 1 indica operação de LEITURA no ADXL345
        let mut buffer = [reg | 0x80, 0]; 
        
        // No SPI, enviamos o endereço e lemos a resposta na mesma transação
        // transfer_in_place modifica o próprio buffer recebendo os dados do chip
        self.spi.transfer_in_place(&mut buffer).await?;
        
        Ok(buffer[1]) // O segundo byte conterá o dado do registrador
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::Error> {
        // Bit 7 (MSB) em 0 indica operação de ESCRITA
        let buffer = [reg & 0x7F, val];
        
        self.spi.write(&buffer).await?;
        Ok(())
    }

    async fn read_multiple(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        // Bit 7 em 1 (Leitura) e Bit 6 em 1 (Leitura múltipla auto-incrementada)
        let cmd = [reg | 0x80 | 0x40];
        
        // Criamos um array de operações: primeiro escreve o comando, depois lê os dados
        let mut operations = [
            SpiOperation::Write(&cmd),
            SpiOperation::Read(buf),
        ];

        // Passamos o array de operações direto para o hardware
        self.spi.transaction(&mut operations).await?;

        Ok(())
    }

    async fn write_multiple(&mut self, reg: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        // Bit 7 em 0 (Escrita) e Bit 6 em 1 (Múltiplos bytes)
        let cmd = [reg | 0x40]; 
        
        // O hardware SPI mantém o pino CS (Chip Select) em nível lógico baixo (ativo) 
        // durante toda a transação, enviando primeiro o comando e depois os bytes de dados.
        let mut operations = [
            SpiOperation::Write(&cmd),
            SpiOperation::Write(bytes),
        ];

        self.spi.transaction(&mut operations).await
    }
}
