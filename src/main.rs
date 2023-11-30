#![no_std]
#![no_main]

use core::{fmt::Debug, time::Duration};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use embedded_hal::digital::v2::{InputPin, OutputPin};

use esp32c3_hal::{
    clock::ClockControl,
    entry,
    gpio::{GpioPin, Output, PushPull},
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::sleep::TimerWakeupSource,
    spi::{
        master::{Instance, Spi},
        FullDuplexMode, SpiMode,
    },
    systimer::SystemTimer,
    Delay, Rtc, IO,
};

use esp_backtrace as _;
use esp_println::println;

use ds18b20::{Ds18b20, Resolution};
use one_wire_bus::{Address, OneWire};
use ssd1351::{
    display::Display,
    interface::SpiInterface,
    mode::{displaymode::DisplayMode, GraphicsMode, RawMode},
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

impl<const S: usize> MeasurementResult<S> {
    pub fn iter(&self) -> core::slice::Iter<'_, (u64, f32)> {
        self.results[..self.result_count].iter()
    }
}

struct DetectedSensors<const S: usize> {
    count: usize,
    addresses: [u64; S],
}

impl<const S: usize> DetectedSensors<S> {
    pub fn iter(&self) -> core::slice::Iter<'_, u64> {
        self.addresses[..self.count].iter()
    }
}

fn discover_sensors<B, E: Debug, const S: usize>(
    bus: &mut OneWire<B>,
    delay: &mut Delay,
) -> DetectedSensors<S>
where
    B: OutputPin<Error = E> + InputPin<Error = E>,
{
    let mut addresses = [0u64; S];
    let mut count = 0;

    for dev in bus.devices(false, delay) {
        match dev {
            Err(err) => {
                println!("ERR: error while detecting connected devices: {:?}", err);
            }
            Ok(addr) => {
                if addr.family_code() != ds18b20::FAMILY_CODE {
                    continue;
                }

                addresses[count] = addr.0;
                count += 1;
            }
        };
    }

    DetectedSensors { count, addresses }
}

fn read_temperatur_measurements<B, E: Debug, const S: usize>(
    devices: DetectedSensors<S>,
    bus: &mut OneWire<B>,
    delay: &mut Delay,
) -> MeasurementResult<S>
where
    B: OutputPin<Error = E> + InputPin<Error = E>,
{
    let mut count = 0;
    let mut results = [(0u64, 0.0f32); S];

    for &addr in devices.iter() {
        let sensor = Ds18b20::new::<core::fmt::Error>(Address(addr))
            .expect("could not create sensor device");
        let sensor_data = sensor
            .read_data(bus, delay)
            .expect("could not read measurement");

        results[count] = (addr, sensor_data.temperature);
        count += 1;

        println!(
            "{}: Device {:?} is at {}°C",
            time(),
            addr,
            sensor_data.temperature
        );
    }

    MeasurementResult {
        result_count: count,
        results,
    }
}

fn setup_display<T: Instance, const DCN: u8>(
    spi: Spi<'_, T, FullDuplexMode>,
    dc: GpioPin<Output<PushPull>, DCN>,
) -> GraphicsMode<SpiInterface<Spi<'_, T, FullDuplexMode>, GpioPin<Output<PushPull>, DCN>>>
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

    // the DC pin tells the SSD1351 controller wether the data coming in via SPI is a command or data
    let dc = io.pins.gpio2.into_push_pull_output();

    let din = io.pins.gpio6;
    let sclk = io.pins.gpio8;
    let cs = io.pins.gpio7;

    // we do not use MISO, since the display will never send us any data,
    // so just use any free GPIO (sadly, it blocks a GPIO)
    let miso = io.pins.gpio3;

    let spi = Spi::new(
        periph.SPI2,
        sclk,
        din,
        miso,
        cs,
        6400u32.kHz(),
        SpiMode::Mode0,
        &clocks,
    );

    let mut display = setup_display(spi, dc);

    let ow_data_pin = io.pins.gpio1.into_open_drain_output();
    let mut bus = one_wire_bus::OneWire::new(ow_data_pin).expect("could not create bus");

    let mut delay = Delay::new(&clocks);

    let mut rtc = Rtc::new(periph.RTC_CNTL);

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::BLUE)
        .build();

    loop {
        println!("{} {} up and runnning!", time(), rtc.get_time_ms());

        let timer = TimerWakeupSource::new(Duration::from_secs(2));

        println!("{}: start measurement", time());

        ds18b20::start_simultaneous_temp_measurement(&mut bus, &mut delay)
            .expect("failed to start measurement");
        Resolution::Bits12.delay_for_measurement_time(&mut delay);

        println!("{}: measurement done", time());

        let sensors: DetectedSensors<4> = discover_sensors(&mut bus, &mut delay);

        let results = read_temperatur_measurements(sensors, &mut bus, &mut delay);

        display.clear();

        let mut buffer = [0u8; 24];
        for (r, (_, temperatur)) in results.iter().enumerate() {
            let pos = Point {
                x: 2,
                y: ((15 * r) + 15) as i32,
            };

            Text::new(
                format(&mut buffer, format_args!("Sensor {}: {}C", r, temperatur)).unwrap(),
                pos,
                text_style,
            )
            .draw(&mut display)
            .expect("could not render text to display");
        }

        println!("{}: sleeping", time());

        rtc.sleep_light(&[&timer], &mut delay);
    }
}
