#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use esp_alloc::export::enumset::enum_set;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::{Priority, software::SoftwareInterruptControl},
    rmt,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{ControllerConfig, Protocol, Protocols, WifiController};
use esp_rtos::embassy::InterruptExecutor;
use flight_esp32s3::{
    dshot::{BoardDshot, Dshot300},
    interrupt_tasks::interrupt_main,
    profiler::Esp32Profiler,
    tasks::{baro_task, drone_state_task, listen_for_commands, telemetry_task},
};
use panic_rtt_target as _;
use static_cell::StaticCell;

static INTERRUPT_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();
static SHARED_SPI: StaticCell<Mutex<NoopRawMutex, Spi<'static, Async>>> = StaticCell::new();
static WIFI_CONTROLLER: StaticCell<WifiController> = StaticCell::new();

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    esp_alloc::heap_allocator!(size: 64 * 1024);

    rtt_target::rtt_init_defmt!();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    let my_mac = esp_hal::efuse::base_mac_address();
    info!("my mac addr: {}", my_mac);

    let mpu_int = Input::new(
        peripherals.GPIO11,
        InputConfig::default().with_pull(esp_hal::gpio::Pull::Down),
    );

    let imu_executor = INTERRUPT_EXECUTOR.init(InterruptExecutor::new(sw_ints.software_interrupt1));
    let imu_spawner = imu_executor.start(Priority::Priority2);

    let spi_config = Config::default()
        .with_frequency(Rate::from_mhz(1))
        .with_mode(Mode::_0);

    let imu_spi = Spi::new(peripherals.SPI2, spi_config)
        .expect("Failed to initialize SPI")
        .with_sck(peripherals.GPIO14)
        .with_mosi(peripherals.GPIO13)
        .with_miso(peripherals.GPIO12);

    let cs_imu = Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());

    let shared_spi = SHARED_SPI.init(Mutex::new(
        Spi::new(peripherals.SPI3, spi_config)
            .expect("Failed to initialize SPI")
            .with_sck(peripherals.GPIO4)
            .with_mosi(peripherals.GPIO5)
            .with_miso(peripherals.GPIO7)
            .into_async(),
    ));

    let cs_baro = Output::new(peripherals.GPIO6, Level::High, OutputConfig::default());

    let wifi_controller = {
        let controller = WIFI_CONTROLLER
            .init(WifiController::new(peripherals.WIFI, ControllerConfig::default()).unwrap());
        controller
            .set_protocols(Protocols::default().with_2_4(enum_set!(Protocol::LR)))
            .unwrap();
        controller
    };

    critical_section::with(|_cs| unsafe {
        embedded_profiling::set_profiler(&Esp32Profiler).unwrap();
    });

    imu_spawner.spawn(interrupt_main(imu_spi, cs_imu, mpu_int).unwrap());
    spawner.spawn(drone_state_task().unwrap());
    spawner.spawn(telemetry_task().unwrap());
    spawner.spawn(baro_task(shared_spi, cs_baro).unwrap());
    spawner.spawn(listen_for_commands(wifi_controller.esp_now()).unwrap());
}
