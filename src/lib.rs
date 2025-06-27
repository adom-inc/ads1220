//! This driver is based on the datasheet which can be found here:
//! https://www.ti.com/lit/ds/symlink/ads1234.pdf?ts=1735781638226

#![no_std]

use embedded_hal::{
    digital::{InputPin, OutputPin},
    spi::SpiBus,
};

use crate::registers::{
    ConfigurationRegister0, ConfigurationRegister1, ConfigurationRegister2, ConfigurationRegister3,
    RegisterDefinition,
};
pub use crate::registers::{
    ConversionMode, DataRate, DrdyMode, FirFilterMode, Gain, IdacCurrent, IdacRoutingDestination,
    InputVoltageSource, LowSidePowerSwitchMode, OperatingMode, ReferenceVoltageSource,
};

mod registers;

/// Driver for the Texas Instrucments ADS1220 Delta-Sigma Analog-to-Digical
/// Converter
pub struct ADS1220<SPI, CS, DRDY> {
    spi: SPI,
    cs: CS,
    drdy: DRDY,
}

#[repr(u8)]
enum StandAloneCommand {
    Reset = 0b0000_0110,
    StartSync = 0b0000_1000,
    PowerDown = 0b0000_0010,
    ReadData = 0b0001_0000,
}

impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY> {
    /// Creates a new ADS1220 driver. This can be used for both the blocking and
    /// async modes. Functions are made available through trait conformity.
    pub fn new(spi: SPI, cs: CS, drdy: DRDY) -> Self {
        Self { spi, cs, drdy }
    }
}

impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    CS: OutputPin,
{
    fn blocking_with_cs<R>(&mut self, f: impl FnOnce(&mut Self) -> R) -> R {
        self.cs.set_low().unwrap();

        let res = f(self);

        self.cs.set_high().unwrap();

        res
    }
}

#[cfg(feature = "embedded-hal-async")]
impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    CS: OutputPin,
{
    async fn with_cs<R>(&mut self, f: impl AsyncFnOnce(&mut Self) -> R) -> R {
        self.cs.set_low().unwrap();

        let res = f(self).await;

        self.cs.set_high().unwrap();

        res
    }
}

impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    SPI: SpiBus,
    CS: OutputPin,
{
    fn blocking_send_stand_alone_command(&mut self, command: StandAloneCommand) {
        self.spi.write(&[command as u8]).unwrap();
        self.spi.flush().unwrap();
    }

    /// Sends a RESET command to the chip which changes all registers back to
    /// their default configuration and throws away any current conversion
    /// results
    pub fn blocking_reset(&mut self) {
        self.blocking_with_cs(|this| {
            this.blocking_send_stand_alone_command(StandAloneCommand::Reset)
        });
    }

    /// Sends a POWERDOWN command to the chip which puts it into it's power down
    /// state to reduce current consumption while idle
    pub fn blocking_power_down(&mut self) {
        self.blocking_with_cs(|this| {
            this.blocking_send_stand_alone_command(StandAloneCommand::PowerDown)
        });
    }

    fn blocking_read_data_inner(&mut self) -> i32 {
        let mut data = [0; 3];

        self.spi.read(&mut data).unwrap();

        i24_to_i32(u32::from_be_bytes([0, data[0], data[1], data[2]]))
    }

    /// Sends an RDATA command and returns the result
    pub fn blocking_read_latest(&mut self) -> i32 {
        self.blocking_with_cs(|this| {
            this.blocking_send_stand_alone_command(StandAloneCommand::ReadData);
            this.blocking_read_data_inner()
        })
    }

    /// Sends a START/SYNC command to the chip which will either trigger a
    /// conversion in single shot mode, or begin continuous conversions
    pub fn blocking_start(&mut self) {
        self.blocking_with_cs(|this| {
            this.blocking_send_stand_alone_command(StandAloneCommand::StartSync)
        });
    }
}


#[cfg(feature = "embedded-hal-async")]
impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: OutputPin,
{
    async fn send_stand_alone_command(&mut self, command: StandAloneCommand) {
        self.spi.write(&[command as u8]).await.unwrap();
        self.spi.flush().await.unwrap();
    }

    /// See [Self::blocking_reset]
    pub async fn reset(&mut self) {
        self.with_cs(async |this| {
            this.send_stand_alone_command(StandAloneCommand::Reset)
                .await
        })
        .await;
    }

    /// See [Self::blocking_power_down]
    pub async fn power_down(&mut self) {
        self.with_cs(async |this| {
            this.send_stand_alone_command(StandAloneCommand::PowerDown)
                .await
        })
        .await;
    }

    async fn read_data_inner(&mut self) -> i32 {
        let mut data = [0; 3];

        self.spi.read(&mut data).await.unwrap();

        i24_to_i32(u32::from_be_bytes([0, data[0], data[1], data[2]]))
    }

    /// See [Self::blocking_read_latest]
    pub async fn read_latest(&mut self) -> i32 {
        self.with_cs(async |this| {
            this.send_stand_alone_command(StandAloneCommand::ReadData)
                .await;
            this.read_data_inner().await
        })
        .await
    }

    /// See [Self::blocking_start]
    pub async fn start(&mut self) {
        self.with_cs(async |this| {
            this.send_stand_alone_command(StandAloneCommand::StartSync)
                .await
        })
        .await;
    }
}

fn i24_to_i32(value: u32) -> i32 {
    // Mask to get the lower 24 bits
    let masked_value = value & 0xFFFFFF;

    // Check if the 24th bit (sign bit) is set
    if masked_value & 0x800000 != 0 {
        // If so, subtract 2^24 to convert to negative
        masked_value as i32 - (1 << 24)
    } else {
        // Otherwise, just return the value as a positive i32
        masked_value as i32
    }
}

impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    SPI: SpiBus,
    CS: OutputPin,
    DRDY: InputPin,
{
    /// Checks if there is currently a conversion available (DRDY is low)
    pub fn is_data_ready(&mut self) -> bool {
        self.drdy.is_low().unwrap()
    }

    /// Waits for data to become ready and then reads from DOUT
    pub fn blocking_read_next(&mut self) -> i32 {
        while !self.is_data_ready() {}

        self.blocking_with_cs(|this| this.blocking_read_data_inner())
    }

    /// Sends a START command and waits for the next available conversion
    pub fn blocking_perform_single_shot_read(&mut self) -> i32 {
        self.blocking_start();
        self.blocking_read_next()
    }
}

#[cfg(feature = "embedded-hal-async")]
impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: OutputPin,
    DRDY: InputPin + embedded_hal_async::digital::Wait,
{
    /// Waits for DRDY to go low signalling that new data is avilable
    pub async fn wait_for_data_ready(&mut self) {
        self.drdy.wait_for_low().await.unwrap()
    }

    /// See [Self::blocking_read_next]
    pub async fn read_next(&mut self) -> i32 {
        self.wait_for_data_ready().await;

        self.with_cs(async |this| this.read_data_inner().await)
            .await
    }

    /// See [Self::blocking_perform_single_shot_read]
    pub async fn perform_single_shot_read(&mut self) -> i32 {
        self.start().await;
        self.read_next().await
    }
}

#[repr(u8)]
enum RegisterCommand {
    Read = 0b0010_0000,
    Write = 0b0100_0000,
}

impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    SPI: SpiBus,
    CS: OutputPin,
    DRDY: InputPin,
{
    fn blocking_read_configuration_register<R: RegisterDefinition>(&mut self) -> R {
        let data = (RegisterCommand::Read as u8) | ((R::register() as u8) << 2);
        let mut result = [0; 1];

        self.spi.write(&[data]).unwrap();
        self.spi.read(&mut result).unwrap();

        result[0].into()
    }

    fn blocking_write_configuration_register<R: RegisterDefinition>(&mut self, value: R) {
        let data = (RegisterCommand::Write as u8) | ((R::register() as u8) << 2);

        self.spi.write(&[data, value.into()]).unwrap();
        self.spi.flush().unwrap();
    }

    fn blocking_update_configuration_register<R: RegisterDefinition>(
        &mut self,
        f: impl FnOnce(R) -> R,
    ) {
        let prev = self.blocking_read_configuration_register::<R>();
        let new = f(prev);

        self.blocking_write_configuration_register::<R>(new);

        debug_assert_eq!(self.blocking_read_configuration_register::<R>(), new)
    }
}

#[cfg(feature = "embedded-hal-async")]
impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: OutputPin,
    DRDY: InputPin,
{
    async fn read_configuration_register<R: RegisterDefinition>(&mut self) -> R {
        let data = (RegisterCommand::Read as u8) | ((R::register() as u8) << 2);
        let mut result = [0; 1];

        self.spi.write(&[data]).await.unwrap();
        self.spi.read(&mut result).await.unwrap();

        result[0].into()
    }

    async fn write_configuration_register<R: RegisterDefinition>(&mut self, value: R) {
        let data = (RegisterCommand::Write as u8) | ((R::register() as u8) << 2);

        self.spi.write(&[data, value.into()]).await.unwrap();
        self.spi.flush().await.unwrap();
    }

    async fn update_configuration_register<R: RegisterDefinition>(
        &mut self,
        f: impl FnOnce(R) -> R,
    ) {
        let prev = self.read_configuration_register::<R>().await;
        let new = f(prev);

        self.write_configuration_register::<R>(new).await;

        debug_assert_eq!(self.read_configuration_register::<R>().await, new)
    }
}

macro_rules! blocking_configuration_function {
    ($name:ident, $ty:ty, $register:ty, $register_field:ident) => {
        paste::paste! {
            #[allow(missing_docs)]
            pub fn [< blocking_get_ $name >](&mut self) -> $ty {
                self.blocking_with_cs(|this| {
                    let curr = this.blocking_read_configuration_register::<$register>();
                    curr.$register_field()
                })
            }
        }

        paste::paste! {
            #[allow(missing_docs)]
            pub fn [< blocking_set_ $name >](&mut self, value: $ty) {
                self.blocking_with_cs(|this| {
                    this.blocking_update_configuration_register::<$register>(|mut prev| {
                        prev.[< set_ $register_field >](value);
                        prev
                    })
                })
            }
        }
    };
}

#[cfg(feature = "embedded-hal-async")]
macro_rules! async_configuration_function {
    ($name:ident, $ty:ty, $register:ty, $register_field:ident) => {
        paste::paste! {
            #[allow(missing_docs)]
            pub async fn [< get_ $name >](&mut self) -> $ty {
                self.with_cs(async |this| {
                    let curr = this.read_configuration_register::<$register>().await;
                    curr.$register_field()
                }).await
            }
        }

        paste::paste! {
            #[allow(missing_docs)]
            pub async fn [< set_ $name >](&mut self, value: $ty) {
                self.with_cs(async |this| {
                    this.update_configuration_register::<$register>(|mut prev| {
                        prev.[< set_ $register_field >](value);
                        prev
                    }).await
                }).await
            }
        }
    };
}

macro_rules! configuration_impl {
    { $(($name:ident, $input:ty, $register:ty, $register_field:ident)),* } => {
        impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
        where
            SPI: SpiBus,
            CS: OutputPin,
            DRDY: InputPin,
        {
            $(

                blocking_configuration_function!(
                    $name, $input, $register, $register_field
                );
            )*
        }

        #[cfg(feature = "embedded-hal-async")]
        impl<SPI, CS, DRDY> ADS1220<SPI, CS, DRDY>
        where
            SPI: embedded_hal_async::spi::SpiBus,
            CS: OutputPin,
            DRDY: InputPin,
        {
            $(
                async_configuration_function!(
                    $name, $input, $register, $register_field
                );
            )*
        }
    };
}

// async and blocking implementations
configuration_impl! {
    // CR0

    (input_voltage_source, InputVoltageSource, ConfigurationRegister0, mux),
    (gain, Gain, ConfigurationRegister0, gain),
    (pga_bypassed, bool, ConfigurationRegister0, pga_bypass),

    // CR1
    (data_rate, DataRate, ConfigurationRegister1, dr),
    (operating_mode, OperatingMode, ConfigurationRegister1, mode),
    (conversion_mode, ConversionMode, ConfigurationRegister1, cm),
    (temperature_sesnor_enabled, bool, ConfigurationRegister1, ts),
    (burn_out_current_sources_enabled, bool, ConfigurationRegister1, bcs),

    // CR2
    (reference_voltage_source, ReferenceVoltageSource, ConfigurationRegister2, vref),
    (fir_filter_mode, FirFilterMode, ConfigurationRegister2, fir),
    (low_side_power_switch_mode, LowSidePowerSwitchMode, ConfigurationRegister2, psw),
    (idac_current, IdacCurrent, ConfigurationRegister2, idac),

    // CR3
    (idac1_destination, IdacRoutingDestination, ConfigurationRegister3, i1mux),
    (idac2_destination, IdacRoutingDestination, ConfigurationRegister3, i2mux),
    (drdy_mode, DrdyMode, ConfigurationRegister3, drdym)
}
