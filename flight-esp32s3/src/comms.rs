use esp_hal::efuse;
use esp_radio::esp_now::{EspNow, EspNowError, EspNowManager, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo};
use flight_core::comms::{CommsError, Message, Receiver, Sender};
use postcard::{from_bytes_cobs, to_slice_cobs};

pub struct BoardComms<'a> {
    rx: BoardRx<'a>,
    tx: BoardTx<'a>,
    manager: EspNowManager<'a>,
}

impl<'a> BoardComms<'a> {
    /// Atomic setup: sets channel and registers peer once
    pub fn new(
        esp_now: EspNow<'a>,
        target_mac: [u8; 6],
        channel: u8,
    ) -> Result<Self, EspNowError> {
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
            manager
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
pub async fn comms_rx_task(mut rx: BoardRx<'static>) {
    loop {
        let data = rx.receive().await.unwrap();
        esp_println::println!("{:?}", data);
    }
}