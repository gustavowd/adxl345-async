# adxl345-async

An async, hardware-agnostic driver for the ADXL345 accelerometer, built on top of `embedded-hal-async`. It supports both I2C and SPI buses via a generic bus trait.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
adxl345-async = "0.1.0"