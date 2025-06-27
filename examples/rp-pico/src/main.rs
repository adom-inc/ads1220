//! This example shows reading 2 resistive bridge load cell sensors using the
//! ADS1220 ADC chip
//! 
//! ADS1220  Pin Configuration:
//! 
//! MISO => GPIO16
//! CS => GPIO17
//! SCLK => GPIO18
//! MOSI => GPIO19
//! DRDY => GPIO20
//! 
//! CLK => GND
//! DVDD => 3V3
//! DGND => GND
//! AVDD => VSYS
//! ASS => GBND
//! 
//! REFP0 => VSYS & Load Cell E+
//! REFN0 => GND & Load Cell E-
//! AIN0 => Load Cell 1 S+
//! AIN1 => Load Cell 1 S-
//! AIN2 => Load Cell 2 S+
//! AIN3 => Load Cell 2 S-

#![no_std]
#![no_main]

use ads1220::ADS1220;
use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

use fugit::RateExtU32;

use bsp::{
    entry,
    hal::{
        Spi,
        clocks::{Clock, init_clocks_and_plls},
        gpio::FunctionSpi,
        pac,
        sio::Sio,
        watchdog::Watchdog,
    },
};

#[entry]
fn main() -> ! {
    defmt::info!("ADS1220 Example!");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = rp_pico::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let spi_sclk = pins.gpio18.into_function::<FunctionSpi>();
    let spi_mosi = pins.gpio19.into_function::<FunctionSpi>();
    let spi_miso = pins.gpio16.into_function::<FunctionSpi>();

    let spi_bus = Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        500_000.Hz(),
        embedded_hal::spi::MODE_1, // MUST be Mode 1
    );
    let spi_cs = pins.gpio17.into_push_pull_output();
    let drdy = pins.gpio20.into_pull_up_input();

    let mut adc = ADS1220::new(spi_bus, spi_cs, drdy);

    adc.blocking_reset();
    delay.delay_ms(10);

    /* Reg 0 */
    adc.blocking_set_input_voltage_source(ads1220::InputVoltageSource::Ain0Ain1);
    adc.blocking_set_gain(ads1220::Gain::X128);
    adc.blocking_set_pga_bypassed(false);

    /* Reg 1 */
    adc.blocking_set_data_rate(ads1220::DataRate::Normal90);
    adc.blocking_set_operating_mode(ads1220::OperatingMode::Normal);
    adc.blocking_set_conversion_mode(ads1220::ConversionMode::Continuous);

    /* Reg 2 */
    adc.blocking_set_reference_voltage_source(ads1220::ReferenceVoltageSource::ExternalRef0);
    adc.blocking_set_fir_filter_mode(ads1220::FirFilterMode::Simultaneous50Hz60Hz);
    adc.blocking_set_low_side_power_switch_mode(ads1220::LowSidePowerSwitchMode::AlwaysOpen);
    adc.blocking_set_idac_current(ads1220::IdacCurrent::Off);

    // Sends a start command to begin conversions
    adc.blocking_start();

    loop {
        let max_value = (2 << 23) as f32;

        adc.blocking_set_input_voltage_source(ads1220::InputVoltageSource::Ain0Ain1);
        let value_1 = adc.blocking_read_next();
        let normalized_1 = value_1 as f32 / max_value;
        let voltage_1 = normalized_1 * 5.0 * 2.0;

        adc.blocking_set_input_voltage_source(ads1220::InputVoltageSource::Ain2Ain3);
        let value_2 = adc.blocking_read_next();
        let normalized_2 = value_2 as f32 / max_value;
        let voltage_2 = normalized_2 * 5.0 * 2.0;

        defmt::info!(
            "value_1 = {}, percent_1 = {}%, voltage_1 = {}V | value_2 = {}, percent_2 = {}%, voltage_2 = {}V",
            value_1,
            normalized_1 * 100.,
            voltage_1,
            value_2,
            normalized_2 * 100.,
            voltage_2,
        );
    }
}
