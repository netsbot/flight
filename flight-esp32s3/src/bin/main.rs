#![feature(duration_millis_float)]
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::time::Duration;
use defmt::{info, println};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::main;
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::time::{Instant, Rate};
use flight_core::fusion::Convention;
use mpu9250::{AccelDataRate, Dlpf, Mpu9250, MpuConfig};
use nalgebra::Vector3;
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let spi_config = Config::default()
        .with_frequency(Rate::from_mhz(1))
        .with_mode(Mode::_3);

    let spi = Spi::new(peripherals.SPI2, spi_config)
        .expect("Failed to initialize SPI")
        .into_async()
        .with_sck(peripherals.GPIO12)
        .with_mosi(peripherals.GPIO11)
        .with_miso(peripherals.GPIO13);

    let mut delay = Delay::new();

    let cs_pin = Output::new(peripherals.GPIO42, Level::High, OutputConfig::default());

    let mut config = MpuConfig::imu();
    config
        .accel_scale(mpu9250::AccelScale::_16G)
        .gyro_scale(mpu9250::GyroScale::_1000DPS)
        .accel_data_rate(AccelDataRate::DlpfConf(Dlpf::_1))
        .gyro_temp_data_rate(mpu9250::GyroTempDataRate::DlpfConf(Dlpf::_1));
    let mut mpu = Mpu9250::imu(spi, cs_pin, &mut delay, &mut config).unwrap();

    let mut fusion = flight_core::fusion::Ahrs::new(
        flight_core::fusion::AhrsConfig::new().with_gyro_range(1000.0),
    );

    let mut last_time = Instant::now();
    let mut update = 0;

    loop {
        let gyro_rad: Vector3<f32> = mpu.gyro().unwrap();
        let gyro = gyro_rad.map(|v| v.to_degrees());
        let accel: Vector3<f32> = mpu.accel().unwrap();

        let sensors = flight_core::Sensors {
            accel,
            gyro,
            magnetometer: None,
            alt: 0.0,
            dt: Duration::from_micros(last_time.elapsed().as_micros()).as_secs_f32(),
        };

        last_time = Instant::now();

        fusion.update(&sensors);
        update += 1;

        if update % 100 == 0 {
            let angle = fusion.euler_angles();
            println!("Roll: {}, Pitch: {}, Yaw: {}", angle.x, angle.y, angle.z);
            update = 0
        }
    }
}
