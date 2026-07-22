use flight_core::fusion::Convention;
use flight_core::{Sensors, fusion};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::Deserialize;
use std::error::Error;
use std::fs::File;
use std::io::Write;

#[derive(Debug, Clone, Copy)]
pub struct SitlData {
    pub sim_time: f64,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub rpy: Vector3<f32>,
}

// Flat helper struct matching CSV headers
#[derive(Debug, Deserialize)]
struct CsvRow {
    sim_time: f64,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    roll_gt: f32,
    pitch_gt: f32,
    yaw_gt: f32,
}

pub fn load_sitl_telemetry(path: &str) -> Result<Vec<SitlData>, Box<dyn Error>> {
    let file = File::open(path)?;
    let mut rdr = csv::Reader::from_reader(file);

    let data = rdr
        .deserialize::<CsvRow>()
        .filter_map(|result| result.ok())
        .map(|row| SitlData {
            sim_time: row.sim_time,
            accel: Vector3::new(row.acc_x, row.acc_y, row.acc_z),
            gyro: Vector3::new(row.gyro_x, row.gyro_y, row.gyro_z),
            rpy: Vector3::new(row.roll_gt, row.pitch_gt, row.yaw_gt),
        })
        .collect();

    Ok(data)
}

fn main() -> Result<(), Box<dyn Error>> {
    let dataset = load_sitl_telemetry("webots/controllers/webots_vehicle/sensor_data.csv")?;
    println!("Successfully loaded {} frames at 500 Hz!", dataset.len());

    if let Some(first) = dataset.first() {
        println!("First frame: {:?}", first);
    }

    let config = fusion::Config::default()
        .with_sample_rate(100.0)
        .with_convention(Convention::Nwu)
        .with_gain(0.5)
        .with_gyro_range(2000.0)
        .with_accel_rejection(10.0)
        .with_magnetic_rejection(10.0)
        .with_rejection_timeout(5.0);
    let mut fusion = fusion::Fusion::new(config);

    // Prepare CSV writer to export per-frame estimates alongside ground-truth.
    // Writes: sim_time, gt_roll, gt_pitch, gt_yaw, est_roll, est_pitch, est_yaw
    let out_path = "results/sitl_estimates.csv";
    // Ensure results dir exists (best-effort)
    let _ = std::fs::create_dir_all("results");
    let out_file = File::create(out_path)?;
    let mut wtr = csv::Writer::from_writer(out_file);
    wtr.write_record(&["sim_time", "est_roll", "est_pitch", "est_yaw"])?;

    for data in dataset {
        fusion.update(Sensors {
            accel: data.accel,
            gyro: data.gyro,
            magnetometer: None,
            alt: 0.0,
        });

        let q = fusion.quaternion();

        // Export CSV row. `data.rpy` contains the ground-truth values read from CSV.
        // `estimated_q.euler_angles()` returns (roll, pitch, yaw) in radians.

        let est_roll = (q.j * q.k + q.w * q.i)
            .atan2(q.w * q.w + q.k * q.k - 0.5)
            .to_degrees();
        let est_pitch = (2.0 * (q.w * q.j - q.i * q.k)).asin().to_degrees();
        let est_yaw = (q.i * q.j + q.w * q.k)
            .atan2(q.w * q.w + q.i * q.i - 0.5)
            .to_degrees();

        wtr.write_record(&[
            data.sim_time.to_string(),
            est_roll.to_string(),
            est_pitch.to_string(),
            est_yaw.to_string(),
        ])?;
    }

    // Flush and close CSV writer
    wtr.flush()?;
    Ok(())
}
