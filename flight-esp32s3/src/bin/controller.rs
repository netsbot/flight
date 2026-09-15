#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_time::{Duration, Instant, Ticker};
use esp_alloc::export::enumset::enum_set;
use esp_hal::{
    clock::CpuClock, interrupt::software::SoftwareInterruptControl, timer::timg::TimerGroup,
};
use esp_radio::wifi::{ControllerConfig, Protocol, Protocols, WifiController};
use flight_core::comms::{Command, Command::Time, Sender};
use flight_esp32s3::comms::BoardTx;
use panic_rtt_target as _;
use static_cell::StaticCell;

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

    let wifi_controller = {
        let controller = WIFI_CONTROLLER
            .init(WifiController::new(peripherals.WIFI, ControllerConfig::default()).unwrap());
        controller
            .set_protocols(Protocols::default().with_2_4(enum_set!(Protocol::LR)))
            .unwrap();
        controller
    };

    let esp_now = wifi_controller.esp_now();
    let (manager, tx, _rx) = esp_now.split();
    manager.set_channel(10).unwrap();

    let target_mac = [172, 39, 110, 170, 188, 84];
    let mut tx = BoardTx::new(tx, target_mac)
        .add_peer(&manager, Some(10))
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_hz(1));

    let time = Instant::now();

    loop {
        let time = Duration::from_nanos(Instant::now().as_nanos() - time.as_nanos());
        match tx.send(Time(time.as_secs() as u32)).await {
            Ok(_) | Err(_) => {}
        }
        ticker.next().await;
    }
}
