import socket
from struct import pack, unpack
from controller import Supervisor, Keyboard


class StandaloneBench:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        # Enable Sensors
        self.accel = self.robot.getDevice("accelerometer")
        self.gyro = self.robot.getDevice("gyro")
        self.imu = self.robot.getDevice("inertial unit")
        self.gps = self.robot.getDevice("gps")

        for s in [self.accel, self.gyro, self.imu, self.gps]:
            s.enable(self.timestep)

        # Motors
        self.motors = [
            self.robot.getDevice("m1_motor"),
            self.robot.getDevice("m2_motor"),
            self.robot.getDevice("m3_motor"),
            self.robot.getDevice("m4_motor")
        ]

        self.max_velocity = 100.0  # Set motor max velocity limit (rad/s or RPM)

        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print("Connecting to Rust Flight Controller on 127.0.0.1:5599...")
            s.connect(("127.0.0.1", 5599))
            print("Connected!")

            while self.robot.step(self.timestep) != -1:
                # Query FRESH telemetry inside the step loop
                sim_time = float(self.robot.getTime())
                acc = self.accel.getValues()
                gyro = self.gyro.getValues()
                alt = self.gps.getValues()[2]

                # Send 32 bytes (7 x f32)
                s.sendall(pack("<8f", sim_time, acc[0], acc[1], acc[2], gyro[0],
                               gyro[1], gyro[2], alt))

                # Receive exactly 16 bytes (4 x f32)
                data = s.recv(16)
                if len(data) < 16:
                    print("Connection closed by Rust server.")
                    break

                cmds = unpack("<4f", data)

                # Set motor velocities safely
                for i, m in enumerate(self.motors):
                    velocity = max(0.0, min(self.max_velocity, cmds[i]))
                    m.setVelocity(velocity)


if __name__ == "__main__":
    bench = StandaloneBench()
    bench.run()