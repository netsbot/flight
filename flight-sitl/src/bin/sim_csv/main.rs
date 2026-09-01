use flight_core::fusion::{self, Convention};
use flight_core::imu::ImuFrame;
use nalgebra::Vector3;
use serde::Deserialize;
use std::error::Error;
use std::fs::File;

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

    let config = fusion::AhrsConfig::default()
        .with_sample_rate(100.0)
        .with_convention(Convention::Nwu)
        .with_gain(0.5)
        .with_gyro_range(2000.0)
        .with_accel_rejection(10.0)
        .with_magnetic_rejection(10.0)
        .with_rejection_timeout(5.0);
    let mut fusion = fusion::Ahrs::new(config);

    // Prepare CSV writer to export per-frame estimates alongside ground-truth.
    // Writes: sim_time, gt_roll, gt_pitch, gt_yaw, est_roll, est_pitch, est_yaw
    let out_path = "results/sitl_estimates.csv";
    // Ensure results dir exists (best-effort)
    let _ = std::fs::create_dir_all("results");
    let out_file = File::create(out_path)?;
    let mut wtr = csv::Writer::from_writer(out_file);
    wtr.write_record(&["sim_time", "est_roll", "est_pitch", "est_yaw"])?;

    let mut prev_time = None;
    for data in dataset {
        let dt = match prev_time {
            Some(prev) => (data.sim_time - prev) as f32,
            None => 0.01,
        };
        prev_time = Some(data.sim_time);

        let frame = ImuFrame {
            accel_ms2: data.accel,
            gyro_rad_s: data.gyro,
            magnetometer: None,
        };
        fusion.update(frame, dt);

        let euler_rad = fusion.euler_angles();
        let est_roll = euler_rad.x.to_degrees();
        let est_pitch = euler_rad.y.to_degrees();
        let est_yaw = euler_rad.z.to_degrees();

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
