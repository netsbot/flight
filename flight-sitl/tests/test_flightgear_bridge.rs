use flight_sitl::{
    FGNetCtrls, FGNetFDM, FlightGearConnection, FG_NET_CTRLS_SIZE, FG_NET_CTRLS_VERSION,
    FG_NET_FDM_SIZE, FG_NET_FDM_VERSION,
};
use std::net::UdpSocket;
use std::thread;
use std::time::Duration;

#[test]
fn test_fg_net_protocols_roundtrip() {
    // 1. FGNetFDM serialization & deserialization
    let mut fdm = FGNetFDM::default();
    fdm.latitude = 0.6565;
    fdm.longitude = -2.1358;
    fdm.altitude = 100.5;
    fdm.phi = 0.05;
    fdm.theta = 0.03;
    fdm.psi = 1.57;
    fdm.vcas = 45.0;

    let fdm_bytes = fdm.to_be_bytes();
    assert_eq!(fdm_bytes.len(), FG_NET_FDM_SIZE);
    let decoded_fdm = FGNetFDM::from_be_bytes(&fdm_bytes).expect("Failed to decode FDM");
    assert_eq!(decoded_fdm.version, FG_NET_FDM_VERSION);
    assert!((decoded_fdm.latitude - fdm.latitude).abs() < 1e-12);
    assert!((decoded_fdm.longitude - fdm.longitude).abs() < 1e-12);
    assert!((decoded_fdm.altitude - fdm.altitude).abs() < 1e-12);
    assert!((decoded_fdm.phi - fdm.phi).abs() < 1e-6);
    assert!((decoded_fdm.theta - fdm.theta).abs() < 1e-6);
    assert!((decoded_fdm.vcas - fdm.vcas).abs() < 1e-6);

    // 2. FGNetCtrls serialization
    let ctrls = FGNetCtrls::new(0.25, -0.15, 0.05, 0.85);
    let ctrls_bytes = ctrls.to_be_bytes();
    assert_eq!(ctrls_bytes.len(), FG_NET_CTRLS_SIZE);
    assert_eq!(&ctrls_bytes[0..4], &FG_NET_CTRLS_VERSION.to_be_bytes());
}

#[test]
fn test_flightgear_connection_loopback() {
    const TEST_FDM_ADDR: &str = "127.0.0.1:15500";
    const TEST_CTRLS_ADDR: &str = "127.0.0.1:15501";

    let mut fg_conn = FlightGearConnection::bind(TEST_FDM_ADDR, TEST_CTRLS_ADDR)
        .expect("Failed to bind FlightGearConnection");

    // Spawn mock FlightGear socket listening for controls
    let ctrls_mock_socket = UdpSocket::bind(TEST_CTRLS_ADDR).expect("Failed to bind mock ctrls");
    ctrls_mock_socket
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();

    // 1. Send controls from Rust -> Mock FG
    fg_conn
        .write_controls(0.12, -0.08, 0.0, 0.55)
        .expect("write_controls failed");

    let mut ctrls_buf = [0u8; 1024];
    let (n, _) = ctrls_mock_socket
        .recv_from(&mut ctrls_buf)
        .expect("Mock FG failed to receive ctrls");
    assert_eq!(n, FG_NET_CTRLS_SIZE);
    assert_eq!(&ctrls_buf[0..4], &FG_NET_CTRLS_VERSION.to_be_bytes());

    // 2. Send mock FDM telemetry from Mock FG -> Rust
    let fdm_sender = UdpSocket::bind("0.0.0.0:0").unwrap();
    let mut mock_fdm = FGNetFDM::default();
    mock_fdm.latitude = 37.618f64.to_radians();
    mock_fdm.longitude = (-122.375f64).to_radians();
    mock_fdm.altitude = 100.0;
    mock_fdm.phi = 0.02;
    mock_fdm.theta = 0.04;
    mock_fdm.psi = 0.0;
    mock_fdm.phidot = 0.001;
    mock_fdm.a_z_pilot = -32.174;
    let mock_bytes = mock_fdm.to_be_bytes();

    fdm_sender
        .send_to(&mock_bytes, TEST_FDM_ADDR)
        .expect("Failed to send mock FDM");

    // Read back in FlightGearConnection
    let (imu, dt, _cur_time, alt, fdm_read) = fg_conn.read().expect("Failed to read FDM packet");
    assert!(dt > 0.0);
    assert!((alt - 100.0).abs() < 1e-3);
    assert!((fdm_read.latitude - mock_fdm.latitude).abs() < 1e-9);
    assert!((imu.accel_ms2.z - (-9.80665)).abs() < 0.1);
}
