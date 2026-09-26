#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::println;
use embassy_futures::join::join3;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker};
use embedded_io_async::{Read, Write};
use esp_alloc::export::enumset::enum_set;
use esp_hal::{
    clock::CpuClock, interrupt::software::SoftwareInterruptControl, timer::timg::TimerGroup,
    usb_serial_jtag::UsbSerialJtag,
};
use esp_radio::{
    esp_now::{EspNowWifiInterface, PeerInfo},
    wifi::{ControllerConfig, Protocol, Protocols, WifiController},
};
use flight_core::comms::Command;
use panic_rtt_target as _;
use postcard::accumulator::{CobsAccumulator, FeedResult};
use static_cell::StaticCell;

static WIFI_CONTROLLER: StaticCell<WifiController> = StaticCell::new();

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(clippy::large_stack_frames)]
#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
    esp_alloc::heap_allocator!(size: 64 * 1024);
    rtt_target::rtt_init_defmt!();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    let my_mac = esp_hal::efuse::base_mac_address();
    println!("my mac addr: {}", my_mac);

    let wifi_controller = {
        let controller = WIFI_CONTROLLER
            .init(WifiController::new(peripherals.WIFI, ControllerConfig::default()).unwrap());
        controller
            .set_protocols(Protocols::default().with_2_4(enum_set!(Protocol::LR)))
            .unwrap();
        controller
    };

    let target_mac = [172, 39, 110, 170, 188, 84];

    let esp_now = wifi_controller.esp_now();
    let (manager, mut tx, mut rx) = esp_now.split();
    manager
        .add_peer(PeerInfo {
            peer_address: target_mac,
            lmk: None,
            channel: Some(10),
            encrypt: false,
            interface: EspNowWifiInterface::Station,
        })
        .unwrap();
    manager.set_channel(10).unwrap();

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let (mut usb_rx, usb_tx) = usb.split();
    let usb_tx = Mutex::<CriticalSectionRawMutex, _>::new(usb_tx);

    // Track drone radio link activity and signal strength
    let drone_link = Mutex::<CriticalSectionRawMutex, Option<(Instant, i8)>>::new(None);

    let esp_to_usb = async {
        loop {
            let received = rx.receive_async().await;

            let rssi = received.info.rx_control.rssi as i8;
            {
                let mut status = drone_link.lock().await;
                *status = Some((Instant::now(), rssi));
            }

            let mut tx_lock = usb_tx.lock().await;
            let _ = tx_lock.write_all(received.data()).await;
        }
    };

    let usb_to_esp = async {
        let mut acc = CobsAccumulator::<256>::new();
        let mut chunk = [0u8; 64];
        let mut send_buf = [0u8; 64];

        loop {
            let Ok(n) = usb_rx.read(&mut chunk).await;
            if n == 0 {
                continue;
            }

            let mut window = &chunk[..n];
            while !window.is_empty() {
                window = match acc.feed::<Command>(window) {
                    FeedResult::Consumed => break,
                    FeedResult::OverFull(rem) => rem,
                    FeedResult::Success { data, remaining } => {
                        if let Ok(slice) = postcard::to_slice_cobs(&data, &mut send_buf) {
                            let _ = tx.send_async(&target_mac, slice).await;
                        }
                        remaining
                    },
                    FeedResult::DeserError(rem) => rem,
                };
            }
        }
    };

    let heartbeat = async {
        let mut ticker = Ticker::every(Duration::from_hz(1));
        let mut buf = [0u8; 32];
        loop {
            ticker.next().await;

            let (drone_linked, rssi) = {
                let status = drone_link.lock().await;
                if let Some((time, rssi)) = *status {
                    if (Instant::now() - time) < Duration::from_secs(2) {
                        (true, Some(rssi))
                    } else {
                        (false, None)
                    }
                } else {
                    (false, None)
                }
            };

            let hb = Command::Pong {
                version: 1,
                drone_linked,
                rssi,
            };

            if let Ok(slice) = postcard::to_slice_cobs(&hb, &mut buf) {
                let mut tx_lock = usb_tx.lock().await;
                let _ = tx_lock.write_all(slice).await;
            }
        }
    };

    join3(esp_to_usb, usb_to_esp, heartbeat).await;
}
