use flight_comms_esp::BoardRx;
use flight_core::comms::Receiver;

#[embassy_executor::task]
pub async fn comms_rx_task(esp_now: esp_radio::esp_now::EspNow<'static>) {
    let (manager, _tx, rx) = esp_now.split();
    manager.set_channel(10).unwrap();
    let target_mac = [172, 39, 110, 170, 188, 84];
    let mut board_rx = BoardRx::new(rx, target_mac);

    loop {
        let data = board_rx.receive().await.unwrap();
        esp_println::println!("{:?}", data);
    }
}
