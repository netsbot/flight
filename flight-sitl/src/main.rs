use nalgebra::Vector3;
use flight_core::FlightComputer;

fn main() {
    let sensor = flight_core::Sensors {
        accel: Vector3::zeros(),
        gyro: Vector3::zeros(),
        magnetometer: None,
        alt: 0.0,
        dt: 0.0,
    };

    let computer = FlightComputer::new();
    computer.update(sensor);
}
