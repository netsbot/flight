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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Event, Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_rtos::embassy::InterruptExecutor;
use flight_core::fusion::{self, AhrsConfig};
use flight_core::imu::{AccumulatedImu, ImuFrame};
use flight_core::RateController;
use flight_esp32s3::imu::BoardImu;
use flight_esp32s3::CycleInstant;
use panic_rtt_target as _;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

static SPI_BUS: StaticCell<Mutex<RefCell<Spi<'static, esp_hal::Blocking>>>> = StaticCell::new();
static IMU_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();

/// Channel sending accumulated IMU delta snapshots from high-priority InterruptExecutor to AHRS loop
static AHRS_CHANNEL: Channel<CriticalSectionRawMutex, AccumulatedImu, 8> = Channel::new();

#[derive(Copy, Clone, Default)]
pub struct ImuProfileStats {
    pub rate_hz: u32,
    pub exec_us: u32,
    pub max_exec_us: u32,
    pub min_exec_us: u32,
    pub loop_dt_us: u32,
}

/// Watch channel publishing latest AHRS attitude (roll, pitch, yaw in degrees) to telemetry / comms
static ATTITUDE_WATCH: Watch<CriticalSectionRawMutex, nalgebra::Vector3<f32>, 2> = Watch::new();

/// Watch channel publishing latest IMU profiling metrics
static PROFILE_WATCH: Watch<CriticalSectionRawMutex, ImuProfileStats, 2> = Watch::new();

static THROTTLE: AtomicI32 = AtomicI32::new(0);

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    rtt_target::rtt_init_defmt!();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // 1. Start esp-rtos scheduler with HW timer and SW interrupt 0 (required for embassy-time / thread scheduler)
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    // 2. Initialize SPI bus and IMU device
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

    let mpu_int = Input::new(
        peripherals.GPIO3,
        InputConfig::default().with_pull(esp_hal::gpio::Pull::Down),
    );

    // 3. Setup InterruptExecutor on software interrupt 1 for high-priority IMU
    let imu_executor = IMU_EXECUTOR.init(InterruptExecutor::new(sw_ints.software_interrupt1));
    let imu_spawner = imu_executor.start(Priority::Priority2);

    // 4. Corrected spawn calls
    imu_spawner.spawn(imu_task(mpu_int, imu).unwrap());
    spawner.spawn(ahrs_task().unwrap());
    spawner.spawn(telemetry_task().unwrap());
}

/// High-priority task running in interrupt context (InterruptExecutor).
/// Handles IMU pin interrupt, reads sensor, steps inner rate PID, and pushes accumulated delta to channel.
#[embassy_executor::task]
async fn imu_task(mut mpu_int: Input<'static>, mut imu: BoardImu) {
    let mut rate_controller = RateController::new();
    let mut last_imu_read = CycleInstant::now();
    let mut accum = AccumulatedImu::ZERO;

    // Profiling variables
    let mut sample_count: u32 = 0;
    let mut profile_window_start = CycleInstant::now();
    let mut total_exec_cycles: u32 = 0;
    let mut max_exec_cycles: u32 = 0;
    let mut min_exec_cycles: u32 = u32::MAX;
    let profile_sender = PROFILE_WATCH.sender();

    // Clear initial interrupt latch
    let _ = imu.read();

    loop {
        // Await hardware rising edge interrupt
        mpu_int.wait_for_high().await;

        let exec_start = CycleInstant::now();

        let Ok(data) = imu.read() else {
            continue;
        };
        
        let dt = last_imu_read.elapsed_secs_and_reset();

        // 1. Accumulate pre-integration delta angles and delta velocities
        accum.delta_angle += data.gyro_rad_s * dt;
        accum.delta_velocity += data.accel_g * dt;
        accum.dt += dt;
        accum.samples += 1;

        // 2. Step 1kHz inner rate controller PID
        rate_controller.step(
            data.gyro_rad_s,
            data.accel_g,
            THROTTLE.load(Ordering::Relaxed) as f32,
            dt,
        );

        // Record execution cycles of handler
        let exec_cycles = exec_start.elapsed_cycles();
        total_exec_cycles += exec_cycles;
        max_exec_cycles = max_exec_cycles.max(exec_cycles);
        min_exec_cycles = min_exec_cycles.min(exec_cycles);
        sample_count += 1;

        // Publish profiling stats periodically (every 500 samples ~0.5s at 1kHz)
        if sample_count >= 500 {
            let window_secs = profile_window_start.elapsed_secs_and_reset();
            let rate_hz = (sample_count as f32 / window_secs) as u32;
            let avg_exec_us = ((total_exec_cycles as f32 / sample_count as f32) * (1.0 / 240.0)) as u32;
            let max_exec_us = (max_exec_cycles / 240) as u32;
            let min_exec_us = (min_exec_cycles / 240) as u32;
            let loop_dt_us = ((window_secs / sample_count as f32) * 1_000_000.0) as u32;

            profile_sender.send(ImuProfileStats {
                rate_hz,
                exec_us: avg_exec_us,
                max_exec_us,
                min_exec_us,
                loop_dt_us,
            });

            sample_count = 0;
            total_exec_cycles = 0;
            max_exec_cycles = 0;
            min_exec_cycles = u32::MAX;
        }

        // Send snapshot every N samples or when enough time elapsed (~5ms / 200Hz)
        if accum.samples >= 5 || accum.dt >= 0.005 {
            let _ = AHRS_CHANNEL.try_send(accum);
            accum = AccumulatedImu::ZERO;
        }
    }
}

/// Outer AHRS attitude estimator task running on normal thread executor.
#[embassy_executor::task]
async fn ahrs_task() {
    let mut ahrs = fusion::Ahrs::new(
        AhrsConfig::default()
            .with_gain(1.5)
            .with_gyro_range(1000.0)
            .with_bias_config(None),
    );

    let sender = ATTITUDE_WATCH.sender();

    loop {
        let accum_data = AHRS_CHANNEL.receive().await;

        if accum_data.dt > 0.0 && accum_data.samples > 0 {
            let avg_gyro = accum_data.delta_angle / accum_data.dt;
            let avg_accel = accum_data.delta_velocity / accum_data.dt;

            let frame = ImuFrame {
                accel_g: avg_accel,
                gyro_rad_s: avg_gyro,
                magnetometer: None,
            };

            ahrs.update(frame, accum_data.dt);
            let euler_deg = ahrs.euler_angles().map(|v| v.to_degrees());
            sender.send(euler_deg);
        }
    }
}

/// Periodic telemetry/logging task.
#[embassy_executor::task]
async fn telemetry_task() {
    let mut attitude_rx = ATTITUDE_WATCH.receiver().unwrap();
    let mut profile_rx = PROFILE_WATCH.receiver().unwrap();
    let mut ticker = Ticker::every(Duration::from_millis(500));

    loop {
        ticker.next().await;

        if let Some(p) = profile_rx.try_get() {
            println!(
                "IMU Rate: {} Hz (dt: {} us) | Exec: avg={} us, min={} us, max={} us",
                p.rate_hz, p.loop_dt_us, p.exec_us, p.min_exec_us, p.max_exec_us
            );
        }

        if let Some(att) = attitude_rx.try_get() {
            let roll = att.x as i32;
            let pitch = att.y as i32;
            let yaw = att.z as i32;
            println!(
                "Attitude (deg): roll={}, pitch={}, yaw={}",
                roll, pitch, yaw
            );
        }
    }
}