import csv
import threading
import queue
import atexit
from controller import Supervisor, Keyboard


class TelemetryLogger:
    """Asynchronous background worker that writes telemetry logs to CSV without stalling the sim."""
    def __init__(self, filename="flight_telemetry.csv"):
        self.filename = filename
        self.log_queue = queue.Queue()
        self.stop_event = threading.Event()

        # Start background writer thread
        self.writer_thread = threading.Thread(target=self._worker, daemon=True)
        self.writer_thread.start()

        # Ensure remaining logs are saved cleanly on script exit
        atexit.register(self.shutdown)

    def log(self, row):
        """Non-blocking log submission (~1 microsecond execution)."""
        self.log_queue.put(row)

    def _worker(self):
        with open(self.filename, mode="w", newline="") as f:
            writer = csv.writer(f)
            # Write Header
            writer.writerow([
                "sim_time",
                "acc_x", "acc_y", "acc_z",
                "gyro_x", "gyro_y", "gyro_z",
                "roll_gt", "pitch_gt", "yaw_gt"
            ])

            while not self.stop_event.is_set() or not self.log_queue.empty():
                try:
                    row = self.log_queue.get(timeout=0.1)
                    writer.writerow(row)
                    self.log_queue.task_done()
                except queue.Empty:
                    continue

    def shutdown(self):
        """Flushes remaining queue items to disk upon exit."""
        self.stop_event.set()
        self.log_queue.join()
        print(f"\n[Logger] Telemetry safely written to {self.filename}")


class StandaloneBench:
    def __init__(self, log_filename="flight_telemetry.csv"):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        # Enable Sensors
        self.accel = self.robot.getDevice("accelerometer")
        self.gyro = self.robot.getDevice("gyro")
        self.imu = self.robot.getDevice("inertial unit")

        for s in [self.accel, self.gyro, self.imu]:
            s.enable(self.timestep)

        # Motors
        self.motors = [
            self.robot.getDevice("m1_motor"),
            self.robot.getDevice("m2_motor"),
            self.robot.getDevice("m3_motor"),
            self.robot.getDevice("m4_motor")
        ]

        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        self.base_omega = 55.37
        self.max_velocity = 100.0

        # Initialize Async Logger
        self.logger = TelemetryLogger(filename=log_filename)

    def run(self):
        print("Standalone Bench Active! Logging directly to CSV...")

        while self.robot.step(self.timestep) != -1:
            key = self.keyboard.getKey()

            pitch_adj = 0.0
            roll_adj = 0.0
            thrust_adj = 0.0

            while key > 0:
                if key == ord('W'): pitch_adj += 5.0
                elif key == ord('S'): pitch_adj -= 5.0
                elif key == ord('A'): roll_adj -= 5.0
                elif key == ord('D'): roll_adj += 5.0
                elif key == ord('Q'): thrust_adj += 10.0
                elif key == ord('E'): thrust_adj -= 10.0
                key = self.keyboard.getKey()

            u = self.base_omega + thrust_adj
            m1 = u + pitch_adj + roll_adj
            m2 = u - pitch_adj - roll_adj
            m3 = u + pitch_adj - roll_adj
            m4 = u - pitch_adj + roll_adj

            cmds = [m1, m2, m3, m4]
            for i, m in enumerate(self.motors):
                m.setVelocity(max(0.0, min(self.max_velocity, cmds[i])))

            # Read raw sensor data
            sim_time = float(self.robot.getTime())
            acc = self.accel.getValues()
            gyro = self.gyro.getValues()
            rpy = self.imu.getRollPitchYaw()

            # Push telemetry directly to CSV logger
            self.logger.log([
                sim_time,
                acc[0], acc[1], acc[2],
                gyro[0], gyro[1], gyro[2],
                rpy[0], rpy[1], rpy[2]
            ])


if __name__ == "__main__":
    bench = StandaloneBench()
    bench.run()