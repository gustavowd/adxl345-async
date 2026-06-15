#![no_std]

use embedded_hal_async::i2c::I2c;


// Registradores do ADXL345
const REG_DEVID: u8 = 0x00;
const REG_BW_RATE: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_DATA_FORMAT: u8 = 0x31;
const REG_DATAX0: u8 = 0x32;

pub enum Address {
   PRIMARY = 0x1D,
   SECONDARY = 0x53
}

pub mod address {
    pub const PRIMARY: u8 = 0x1D;
    pub const SECONDARY: u8 = 0x53;
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

pub struct Adxl345Async<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Adxl345Async<I2C>
where
    I2C: I2c,
{
    /// Cria uma nova instância do driver
    pub fn new(i2c: I2C, addr: Option<Address>) -> Self {
        Self {
            i2c,
            address: addr.map(|a| a as u8).unwrap_or(address::SECONDARY),
        }
    }

    /// Destrói o driver e devolve a instância do I2C
    pub fn release(self) -> I2C {
        self.i2c
    }

    // --- FUNÇÕES INTERNAS DE COMUNICAÇÃO (AGORA ASYNC) ---

    async fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[reg, val]).await?;
        Ok(())
    }

    // --- MÉTODOS PÚBLICOS DO SENSOR ---

    /// Lê o ID do dispositivo (Deve retornar 0xE5 se a comunicação estiver OK)
    pub async fn get_device_id(&mut self) -> Result<u8, I2C::Error> {
        self.read_reg(REG_DEVID).await
    }

    /// Inicializa o sensor tirando-o do modo de standby e colocando em modo de medição
    pub async fn setup(&mut self) -> Result<(), I2C::Error> {
        // Coloca em modo de medição (Measurement Mode)
        self.write_reg(REG_POWER_CTL, 0x08).await?;
        Ok(())
    }

    /// Configura a escala de leitura (G-Range)
    pub async fn set_range(&mut self, range: Range) -> Result<(), I2C::Error> {
        let format = range as u8;
        self.write_reg(REG_DATA_FORMAT, format).await?;
        Ok(())
    }

    /// Configura a taxa de amostragem de dados (Data Rate)
    pub async fn set_data_rate(&mut self, rate: DataRate) -> Result<(), I2C::Error> {
        self.write_reg(REG_BW_RATE, rate as u8).await?;
        Ok(())
    }

    /// Lê os três eixos (X, Y, Z) de aceleração brutos de uma única vez
    pub async fn read_accel(&mut self) -> Result<(i16, i16, i16), I2C::Error> {
        let mut buf = [0u8; 6];
        // O ADXL345 requer leitura múltipla começando no REG_DATAX0
        self.i2c.write_read(self.address, &[REG_DATAX0], &mut buf).await?;

        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }
}
