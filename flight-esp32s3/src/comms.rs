use core::marker::PhantomData;

use esp_radio::esp_now::{
    EspNowError, EspNowManager, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo,
};
use flight_core::comms::{Command, CommsError, Receiver, Sender};
use postcard::from_bytes;

pub struct BoardRx<'a> {
    rx: EspNowReceiver<'a>,
    my_mac: [u8; 6],
}

impl<'a> BoardRx<'a> {
    pub fn new(rx: EspNowReceiver<'a>, my_addr: [u8; 6]) -> Self {
        Self {
            rx,
            my_mac: my_addr,
        }
    }
}

impl<'a> Receiver for BoardRx<'a> {
    async fn receive(&mut self) -> Result<Command, CommsError> {
        loop {
            let r = self.rx.receive_async().await;
            if r.info.dst_address != self.my_mac {
                continue;
            }

            let Ok(command) = from_bytes(r.data()) else {
                continue;
            };

            esp_println::println!("{}", r.info.rx_control.rssi);

            return Ok(command);
        }
    }
}

/// Typestate marker: Peer has not yet been registered.
pub struct Unconfigured;

/// Typestate marker: Peer is registered and ready for transmission.
pub struct PeerAdded;

pub struct BoardTx<'a, State = Unconfigured> {
    tx: EspNowSender<'a>,
    target_mac: [u8; 6],
    _state: PhantomData<State>,
}

impl<'a> BoardTx<'a, Unconfigured> {
    /// Create an unconfigured `BoardTx`. Peer must be added via `.add_peer(...)` before sending.
    pub fn new(tx: EspNowSender<'a>, target_mac: [u8; 6]) -> Self {
        Self {
            tx,
            target_mac,
            _state: PhantomData,
        }
    }

    /// Registers the target peer in `EspNowManager` and transitions into `BoardTx<PeerAdded>`.
    pub fn add_peer(
        self,
        manager: &EspNowManager<'_>,
        channel: Option<u8>,
    ) -> Result<BoardTx<'a, PeerAdded>, EspNowError> {
        manager.add_peer(PeerInfo {
            peer_address: self.target_mac,
            lmk: None,
            channel,
            encrypt: false,
            interface: EspNowWifiInterface::Station,
        })?;

        Ok(BoardTx {
            tx: self.tx,
            target_mac: self.target_mac,
            _state: PhantomData,
        })
    }
}

impl<'a> BoardTx<'a, PeerAdded> {
    /// Directly construct a `BoardTx` and register the peer in one step.
    pub fn new_with_peer(
        tx: EspNowSender<'a>,
        manager: &EspNowManager<'_>,
        target_mac: [u8; 6],
        channel: Option<u8>,
    ) -> Result<Self, EspNowError> {
        BoardTx::new(tx, target_mac).add_peer(manager, channel)
    }

    /// Construct `BoardTx` assuming the peer was ALREADY added beforehand.
    /// Verifies at runtime that the peer exists in `EspNowManager`.
    pub fn from_existing_peer(
        tx: EspNowSender<'a>,
        manager: &EspNowManager<'_>,
        target_mac: [u8; 6],
    ) -> Result<Self, CommsError> {
        if !manager.peer_exists(&target_mac) {
            return Err(CommsError::NotConnected);
        }

        Ok(Self {
            tx,
            target_mac,
            _state: PhantomData,
        })
    }

    /// Unchecked construction if you have already added the peer manually.
    pub unsafe fn new_unchecked(tx: EspNowSender<'a>, target_mac: [u8; 6]) -> Self {
        Self {
            tx,
            target_mac,
            _state: PhantomData,
        }
    }
}

impl<'a> Sender for BoardTx<'a, PeerAdded> {
    async fn send(&mut self, command: Command) -> Result<(), CommsError> {
        let mut buf = [0u8; 64];

        let serialized = postcard::to_slice(&command, &mut buf).map_err(CommsError::from)?;

        self.tx
            .send_async(&self.target_mac, serialized)
            .await
            .map_err(|_| CommsError::Io)?;

        Ok(())
    }
}

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

