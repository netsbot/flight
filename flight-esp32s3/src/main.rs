#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cell::RefCell;
use core::sync::atomic::{AtomicI32, Ordering};
use critical_section::Mutex;
use defmt::println;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Event, Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::spi::master::{Config, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use flight_core::fusion::{self, AhrsConfig};
use flight_core::imu::{AccumulatedImu, ImuFrame};
use flight_core::RateController;
use flight_esp32s3::imu::BoardImu;
use flight_esp32s3::CycleInstant;
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

static SPI_BUS: static_cell::StaticCell<Mutex<RefCell<Spi<'static, esp_hal::Blocking>>>> =
    static_cell::StaticCell::new();

/// Accumulator holding motion deltas from ISR for outer AHRS loop
static ACCUMULATOR: Mutex<RefCell<AccumulatedImu>> =
    Mutex::new(RefCell::new(AccumulatedImu::ZERO));

static IMU_INT_PIN: Mutex<RefCell<Option<Input<'static>>>> = Mutex::new(RefCell::new(None));
static IMU: Mutex<RefCell<Option<BoardImu>>> = Mutex::new(RefCell::new(None));
static THROTTLE: AtomicI32 = AtomicI32::new(0);
static RATE_CONTROLLER: Mutex<RefCell<Option<RateController>>> = Mutex::new(RefCell::new(None));
static LAST_IMU_READ: Mutex<RefCell<Option<CycleInstant>>> = Mutex::new(RefCell::new(None));

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_hal::main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let delay = Delay::new();

    let imu = {
        let spi_config = Config::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0);
        let spi = Spi::new(peripherals.SPI2, spi_config)
            .expect("Failed to initialize SPI")
            .with_sck(peripherals.GPIO12)
            .with_mosi(peripherals.GPIO11)
            .with_miso(peripherals.GPIO13);
        let spi_bus = SPI_BUS.init(Mutex::new(RefCell::new(spi)));
        let cs_pin = Output::new(peripherals.GPIO42, Level::High, OutputConfig::default());
        BoardImu::new(spi_bus, cs_pin)
    };

    let mut ahrs = fusion::Ahrs::new(
        AhrsConfig::default()
            .with_gyro_range(1000.0)
            .with_gain(1.5)
            .with_bias_config(None),
    );

    let mut mpu_int = Input::new(
        peripherals.GPIO3,
        InputConfig::default().with_pull(esp_hal::gpio::Pull::Down),
    );
    mpu_int.listen(Event::RisingEdge);

    critical_section::with(|cs| {
        *IMU_INT_PIN.borrow_ref_mut(cs) = Some(mpu_int);
        *IMU.borrow_ref_mut(cs) = Some(imu);
        *RATE_CONTROLLER.borrow_ref_mut(cs) = Some(RateController::new());
        *LAST_IMU_READ.borrow_ref_mut(cs) = Some(CycleInstant::now());
    });
    
    esp_hal::gpio::Io::new(peripherals.IO_MUX).set_interrupt_handler(gpio_handler);

    critical_section::with(|cs| {
        let mut imu_ref = IMU.borrow_ref_mut(cs);
        if let Some(imu) = imu_ref.as_mut() {
            let _ = imu.read();
        }
    });

    let mut last_log = esp_hal::time::Instant::now();

    loop {
        // Drain accumulated motion snapshot and reset accumulator atomically
        let accum_data = critical_section::with(|cs| {
            let mut accum = ACCUMULATOR.borrow_ref_mut(cs);
            let snapshot = *accum;
            *accum = AccumulatedImu::ZERO;
            snapshot
        });

        if accum_data.dt > 0.0 && accum_data.samples > 0 {
            // Average angular rate & acceleration over accumulated period
            let avg_gyro = accum_data.delta_angle / accum_data.dt;
            let avg_accel = accum_data.delta_velocity / accum_data.dt;

            let frame = ImuFrame {
                accel_g: avg_accel,
                gyro_rad_s: avg_gyro,
                magnetometer: None,
            };

            // Run AHRS update ONCE for the accumulated interval
            ahrs.update(frame, accum_data.dt);
        }

        // Periodically log attitude
        if last_log.elapsed().as_millis() >= 200 {
            let euler_deg = ahrs.euler_angles().map(|v| v.to_degrees());
            println!("Attitude: {:?}", euler_deg);
            last_log = esp_hal::time::Instant::now();
        }

        delay.delay_millis(5);
    }
}

#[esp_hal::handler]
fn gpio_handler() {
    critical_section::with(|cs| {
        let mut pin_ref = IMU_INT_PIN.borrow_ref_mut(cs);
        let Some(pin) = pin_ref.as_mut() else { return };

        if !pin.is_interrupt_set() {
            return;
        }
        pin.clear_interrupt();

        let mut imu_ref = IMU.borrow_ref_mut(cs);
        let Some(imu) = imu_ref.as_mut() else { return };

        let Ok(data) = imu.read() else { return };

        let mut last_imu_read_ref = LAST_IMU_READ.borrow_ref_mut(cs);
        let Some(last_imu_read) = last_imu_read_ref.as_mut() else { return };
        let dt = last_imu_read.elapsed_secs_and_reset();

        // 1. Accumulate pre-integration delta angles and delta velocities
        let mut accum = ACCUMULATOR.borrow_ref_mut(cs);
        accum.delta_angle += data.gyro_rad_s * dt;
        accum.delta_velocity += data.accel_g * dt;
        accum.dt += dt;
        accum.samples += 1;

        // 2. Step 1kHz inner rate controller PID
        let mut rate_controller_ref = RATE_CONTROLLER.borrow_ref_mut(cs);
        if let Some(rate_controller) = rate_controller_ref.as_mut() {
            // TODO: need to properly map u32 to throttle range
            rate_controller.step(
                data.gyro_rad_s,
                data.accel_g,
                THROTTLE.load(Ordering::Relaxed) as f32,
                dt,
            );
        }
    });
}
