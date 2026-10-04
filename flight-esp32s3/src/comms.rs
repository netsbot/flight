use core::sync::atomic::Ordering;

use embassy_time::{Duration, Ticker};
use esp_hal::efuse;
use esp_radio::esp_now::{
    EspNow, EspNowError, EspNowManager, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo,
};
use flight_core::{
    Setpoint,
    comms::{CommsError, Message, Receiver, Sender},
};
use nalgebra::Vector3;
use postcard::{from_bytes_cobs, to_slice_cobs};

use crate::{SET_POINT_CHANNEL, STATE_WATCH, THROTTLE};

pub struct BoardComms<'a> {
    rx: BoardRx<'a>,
    tx: BoardTx<'a>,
    manager: EspNowManager<'a>,
}

impl<'a> BoardComms<'a> {
    /// Atomic setup: sets channel and registers peer once
    pub fn new(esp_now: EspNow<'a>, target_mac: [u8; 6], channel: u8) -> Result<Self, EspNowError> {
        let (manager, tx, rx) = esp_now.split();
        manager.set_channel(channel)?;
        manager.add_peer(PeerInfo {
            peer_address: target_mac,
            lmk: None,
            channel: Some(channel),
            encrypt: false,
            interface: EspNowWifiInterface::Station,
        })?;

        let my_mac: [u8; 6] = efuse::base_mac_address().as_bytes().try_into().unwrap();
        Ok(Self {
            rx: BoardRx { rx, my_mac },
            tx: BoardTx { tx, target_mac },
            manager,
        })
    }

    pub fn split(self) -> (BoardRx<'a>, BoardTx<'a>, EspNowManager<'a>) {
        (self.rx, self.tx, self.manager)
    }
}

pub struct BoardRx<'a> {
    rx: EspNowReceiver<'a>,
    my_mac: [u8; 6],
}

impl<'a> Receiver for BoardRx<'a> {
    async fn receive(&mut self) -> Result<Message, CommsError> {
        loop {
            let r = self.rx.receive_async().await;
            if r.info.dst_address != self.my_mac {
                continue;
            }

            let mut buf = [0u8; 64];
            let data = r.data();
            if data.len() > buf.len() {
                continue;
            }
            buf[..data.len()].copy_from_slice(data);

            let Ok(command) = from_bytes_cobs(&mut buf[..data.len()]) else {
                continue;
            };

            return Ok(command);
        }
    }
}

pub struct BoardTx<'a> {
    tx: EspNowSender<'a>,
    target_mac: [u8; 6],
}

impl<'a> Sender for BoardTx<'a> {
    async fn send(&mut self, message: Message) -> Result<(), CommsError> {
        let mut buf = [0u8; 64];

        let serialized = to_slice_cobs(&message, &mut buf).map_err(CommsError::from)?;

        self.tx
            .send_async(&self.target_mac, serialized)
            .await
            .map_err(|_| CommsError::Io)?;

        Ok(())
    }
}

#[embassy_executor::task]
pub async fn comms_rx_task(mut board_rx: BoardRx<'static>) {
    let setpoint_sender = SET_POINT_CHANNEL.sender();

    loop {
        // TODO: go to home when out of range
        let Ok(msg) = board_rx.receive().await else {
            continue;
        };

        match msg {
            Message::RollRate(data) => {
                setpoint_sender.send(Setpoint::RollRate(<Vector3<f32>>::from(data)))
            }
            Message::Throttle(throttle) => THROTTLE.store(throttle, Ordering::Relaxed),
            Message::Telemetry { .. } => {}
            Message::Pong { .. } => {}
        }
    }
}

#[embassy_executor::task]
pub async fn comms_tx_task(mut board_tx: BoardTx<'static>) {
    let mut telemetry_ticker = Ticker::every(Duration::from_hz(25));
    let mut state_receiver = STATE_WATCH.receiver().unwrap();
    loop {
        let state = state_receiver.get().await;

        // TODO: go to home when out of range
        let _ = board_tx
            .send(Message::Telemetry {
                altitude: state.altitude,
                attitude: <[f32; 3]>::from(state.attitude),
                coords: [0.0, 0.0],
            })
            .await;

        telemetry_ticker.next().await
    }
}
