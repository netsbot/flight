import math
import random
import socket
from struct import pack, unpack
from controller import Supervisor, Keyboard


class RealisticWindSimulator:
    """Simulates vertical updrafts/downdrafts + tipping torques (roll/pitch perturbations)."""
    def __init__(self, air_density=1.225):
        self.air_density = air_density  # kg/m^3
        self.drag_area_xy = 0.015       # Small horizontal area (m^2) to prevent excessive drift
        self.drag_area_z = 0.09         # Large vertical flat-plate area (m^2) for strong draft coupling
        
        # Continuous turbulence (OU process)
        self.turbulence = [0.0, 0.0, 0.0]
        self.tau = 1.5
        self.sigma_h = 0.2              # Horizontal turbulence (m/s)
        self.sigma_v = 0.8              # Strong vertical turbulence (m/s)

        # Vertical Updraft / Downdraft gust state
        self.v_gust_active = False
        self.v_gust_start = 0.0
        self.v_gust_duration = 2.0
        self.v_gust_peak = 0.0
        self.next_v_gust_time = 3.0

        # Tipping torque gust state (roll & pitch disturbance moments)
        self.tip_gust_active = False
        self.tip_gust_start = 0.0
        self.tip_gust_duration = 1.0
        self.tip_torque_peak = [0.0, 0.0]  # [Tau_roll, Tau_pitch] in N*m
        self.next_tip_time = 2.5

    def step(self, sim_time, dt, drone_velocity_world):
        sqrtdt = math.sqrt(dt) if dt > 0 else 0.0
        decay = math.exp(-dt / self.tau)

        # 1. Update continuous turbulence
        self.turbulence[0] = self.turbulence[0] * decay + self.sigma_h * sqrtdt * random.gauss(0, 1)
        self.turbulence[1] = self.turbulence[1] * decay + self.sigma_h * sqrtdt * random.gauss(0, 1)
        self.turbulence[2] = self.turbulence[2] * decay + self.sigma_v * sqrtdt * random.gauss(0, 1)

        # 2. Vertical updraft / downdraft bursts (up to ±2.5 m/s)
        if not self.v_gust_active and sim_time >= self.next_v_gust_time:
            self.v_gust_active = True
            self.v_gust_start = sim_time
            self.v_gust_duration = random.uniform(1.5, 3.0)
            self.v_gust_peak = random.choice([-1.0, 1.0]) * random.uniform(1.2, 2.5)

        v_gust_z = 0.0
        if self.v_gust_active:
            elapsed = sim_time - self.v_gust_start
            if elapsed < self.v_gust_duration:
                v_gust_z = 0.5 * self.v_gust_peak * (1.0 - math.cos(2.0 * math.pi * elapsed / self.v_gust_duration))
            else:
                self.v_gust_active = False
                self.next_v_gust_time = sim_time + random.uniform(3.0, 6.0)

        # 3. Tipping torque gusts (direct roll/pitch moments ±0.06 to 0.12 N*m)
        if not self.tip_gust_active and sim_time >= self.next_tip_time:
            self.tip_gust_active = True
            self.tip_gust_start = sim_time
            self.tip_gust_duration = random.uniform(0.8, 1.8)
            self.tip_torque_peak = [
                random.gauss(0, 0.08),  # Roll torque (N*m)
                random.gauss(0, 0.08)   # Pitch torque (N*m)
            ]

        torque_moment = [0.0, 0.0, 0.0]
        if self.tip_gust_active:
            elapsed = sim_time - self.tip_gust_start
            if elapsed < self.tip_gust_duration:
                factor = 0.5 * (1.0 - math.cos(2.0 * math.pi * elapsed / self.tip_gust_duration))
                torque_moment = [
                    self.tip_torque_peak[0] * factor,
                    self.tip_torque_peak[1] * factor,
                    0.0
                ]
            else:
                self.tip_gust_active = False
                self.next_tip_time = sim_time + random.uniform(2.5, 5.0)

        # 4. Total wind velocity
        v_wind = [
            self.turbulence[0],
            self.turbulence[1],
            self.turbulence[2] + v_gust_z
        ]

        # 5. Compute directional drag forces (separate XY vs Z drag area)
        v_rel = [w - d for w, d in zip(v_wind, drone_velocity_world[:3])]
        speed_xy = math.hypot(v_rel[0], v_rel[1])
        force_x = 0.5 * self.air_density * self.drag_area_xy * speed_xy * v_rel[0]
        force_y = 0.5 * self.air_density * self.drag_area_xy * speed_xy * v_rel[1]
        force_z = 0.5 * self.air_density * self.drag_area_z * abs(v_rel[2]) * v_rel[2]

        force = [force_x, force_y, force_z]
        return force, torque_moment


class StandaloneBench:
    def __init__(self):
        self.robot = Supervisor()
        self.drone_node = self.robot.getSelf()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0

        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        # Enable Sensors
        self.accel = self.robot.getDevice("accelerometer")
        self.gyro = self.robot.getDevice("gyro")
        self.imu = self.robot.getDevice("inertial unit")
        self.altimeter = self.robot.getDevice("altimeter")

        for s in [self.accel, self.gyro, self.imu, self.altimeter]:
            s.enable(self.timestep)

        # Motors
        self.motors = [
            self.robot.getDevice("m1_motor"),
            self.robot.getDevice("m2_motor"),
            self.robot.getDevice("m3_motor"),
            self.robot.getDevice("m4_motor")
        ]

        self.max_velocity = 100.0

        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        # Updraft/Downdraft & Tipping Torque simulator
        self.wind_sim = RealisticWindSimulator()

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print("Connecting to Rust Flight Controller on 127.0.0.1:5599...")
            s.connect(("127.0.0.1", 5599))
            print("Connected! Updraft/Downdraft & Tipping Torque simulation active.")

            while self.robot.step(self.timestep) != -1:
                sim_time = float(self.robot.getTime())

                # 1. Compute and apply vertical drafts + tipping torques
                drone_vel = self.drone_node.getVelocity()
                wind_force, wind_torque = self.wind_sim.step(sim_time, self.dt, drone_vel)
                
                # Apply aerodynamic force at CG (world frame)
                self.drone_node.addForce(wind_force, False)
                # Apply tipping disturbance torque on Roll & Pitch (world frame)
                self.drone_node.addTorque(wind_torque, False)

                # 2. Query FRESH telemetry inside step loop
                acc = self.accel.getValues()
                gyro = self.gyro.getValues()
                alt = self.altimeter.getValue()
                                
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