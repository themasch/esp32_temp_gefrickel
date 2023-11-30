#![no_std]
#![no_main]

use core::{fmt::Debug, time::Duration};

use ds18b20::{Ds18b20, Resolution};
use embedded_hal::{
    blocking::delay::DelayMs,
    digital::v2::{InputPin, OutputPin},
};
use esp32c3_hal::{
    self as hal,
    clock::{ClockControl, Clocks},
    entry,
    peripherals::Peripherals,
    prelude::*,
    reset::get_reset_reason,
    rtc_cntl::sleep::TimerWakeupSource,
    spi::{master::Spi, SpiMode},
    systimer::SystemTimer,
    Delay, Rtc, IO,
};
use esp_backtrace as _;
use esp_println::{dbg, println};
use hal::{
    gpio::{GpioPin, Output, Pins, PushPull},
    peripheral::Peripheral,
    peripherals::SPI2,
    reset::get_wakeup_cause,
    rtc_cntl::SocResetReason,
    spi::{master::Instance, FullDuplexMode},
};
use one_wire_bus::{OneWire, OneWireError};

use ssd1351::{
    display::Display,
    interface::{DisplayInterface, SpiInterface},
    mode::{displaymode::DisplayMode, GraphicsMode, RawMode},
};

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_6X10, FONT_9X18_BOLD},
        MonoTextStyleBuilder,
    },
    pixelcolor::{BinaryColor, Rgb565},
    prelude::*,
    text::{Alignment, Text},
};

mod efmt;

use efmt::format;

fn time() -> f64 {
    SystemTimer::now() as f64 / SystemTimer::TICKS_PER_SECOND as f64
}

#[derive(Debug)]
struct MeasurementResult<const S: usize> {
    result_count: usize,
    results: [(u64, f32); S],
}

fn get_current_measurements<B, E, const S: usize>(
    bus: &mut OneWire<B>,
    delay: &mut Delay,
) -> MeasurementResult<S>
where
    B: OutputPin<Error = E>,
    B: InputPin<Error = E>,
    E: Debug,
{
    let mut search_state = None;
    let mut count = 0;
    let mut results = [(0u64, 0.0f32); S];
    loop {
        let result = bus.device_search(search_state.as_ref(), false, delay);

        if let Err(err) = result {
            println!("ERR: {:?}", err);
            break;
        }

        if let Some((dev_addr, state)) = result.unwrap() {
            search_state = Some(state);
            if dev_addr.family_code() != ds18b20::FAMILY_CODE {
                continue;
            }

            let sensor =
                Ds18b20::new::<core::fmt::Error>(dev_addr).expect("could not create sensor device");

            let sensor_data = sensor
                .read_data(bus, delay)
                .expect("could not read measurement");

            results[count] = (dev_addr.0, sensor_data.temperature);
            count += 1;
            println!(
                "{}: Device {:?} is at {}°C",
                time(),
                dev_addr,
                sensor_data.temperature
            );
        } else {
            break;
        }
    }

    MeasurementResult {
        result_count: count,
        results,
    }
}

fn setup_display<'d, T: Instance, const DCN: u8>(
    spi: Spi<'d, T, FullDuplexMode>,
    dc: GpioPin<Output<PushPull>, DCN>,
) -> GraphicsMode<SpiInterface<Spi<'d, T, FullDuplexMode>, GpioPin<Output<PushPull>, DCN>>>
where
    GpioPin<Output<PushPull>, DCN>: OutputPin,
{
    let interface = SpiInterface::new(spi, dc);

    let mut display = Display::new(
        interface,
        ssd1351::properties::DisplaySize::Display128x128,
        ssd1351::properties::DisplayRotation::Rotate0,
    );

    display.init().expect("could not init display");

    DisplayMode::<RawMode<_>>::new(display).into::<_, GraphicsMode<_>>()
}

#[entry]
fn main() -> ! {
    let periph = Peripherals::take();
    let system = periph.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = IO::new(periph.GPIO, periph.IO_MUX);
    let ow_data_pin = io.pins.gpio1.into_open_drain_output();

    let dc = io.pins.gpio2.into_push_pull_output();

    let din = io.pins.gpio6;
    let miso = io.pins.gpio3;
    let sclk = io.pins.gpio8;
    let cs = io.pins.gpio7;

    let spi = Spi::new(
        periph.SPI2,
        sclk,
        din,
        miso,
        cs,
        12800u32.kHz(),
        SpiMode::Mode0,
        &clocks,
    );

    let mut display = setup_display(spi, dc);
    let mut bus = one_wire_bus::OneWire::new(ow_data_pin).expect("could not create bus");
    let mut delay = Delay::new(&clocks);

    let mut rtc = Rtc::new(periph.RTC_CNTL);

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::BLUE)
        .build();

    loop {
        println!("{} {} up and runnning!", time(), rtc.get_time_ms());
        let reason = get_reset_reason().unwrap_or(SocResetReason::ChipPowerOn);
        println!("reset reason: {:?}", reason);
        let wake_reason = get_wakeup_cause();
        println!("wake reason: {:?}", wake_reason);

        let timer = TimerWakeupSource::new(Duration::from_secs(2));

        println!("{}: start measurement", time());

        ds18b20::start_simultaneous_temp_measurement(&mut bus, &mut delay)
            .expect("failed to start measurement");
        Resolution::Bits12.delay_for_measurement_time(&mut delay);

        println!("{}: measurement done", time());

        let results: MeasurementResult<4> = get_current_measurements(&mut bus, &mut delay);
        dbg!(&results);

        display.clear();
        let mut buffer = [0u8; 64];
        for r in 0..results.result_count {
            Text::new(
                format(
                    &mut buffer,
                    format_args!("Sensor {}: {}C", r, &results.results[r].1),
                )
                .unwrap(),
                Point {
                    x: 2,
                    y: ((15 * r) + 15) as i32,
                },
                text_style,
            )
            .draw(&mut display).expect("could not render text to display");
        }

        println!("{}: sleeping", time());

        rtc.sleep_light(&[&timer], &mut delay);
    }
}
