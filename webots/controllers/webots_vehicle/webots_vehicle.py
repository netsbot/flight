import os
import csv
import math
from controller import Supervisor, Keyboard


class StandaloneBench:
    def __init__(self, log_filename="flight_telemetry.csv"):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        # Devices
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

        # Gentle settings
        self.angle_step = math.radians(1.0)        # 1 deg per key press
        self.max_target_angle = math.radians(20.0) # 20 deg max
        
        # Soft gain
        self.kp_angle = 3.0

        # Logging
        self.log_path = os.path.abspath(log_filename)
        print(f"\n[Logger] Writing synchronously to: {self.log_path}\n")

    def run(self):
        with open(self.log_path, mode="w", newline="", buffering=1) as f:
            writer = csv.writer(f)
            
            writer.writerow([
                "sim_time",
                "acc_x", "acc_y", "acc_z",
                "gyro_x", "gyro_y", "gyro_z",
                "roll_gt", "pitch_gt", "yaw_gt"
            ])

            print("Standalone Bench Active (Dual Axis Debugging Enabled)!")
            print("Controls: W/S (Pitch ±1°) | A/D (Roll ±1°) | R (Reset Level)")

            target_roll = 0.0
            target_pitch = 0.0
            step_count = 0

            while self.robot.step(self.timestep) != -1:
                step_count += 1
                key = self.keyboard.getKey()
                thrust_adj = 0.0

                while key > 0:
                    if key == ord('W'):
                        target_pitch = min(self.max_target_angle, target_pitch + self.angle_step)
                    elif key == ord('S'):
                        target_pitch = max(-self.max_target_angle, target_pitch - self.angle_step)
                    elif key == ord('A'):
                        target_roll = max(-self.max_target_angle, target_roll - self.angle_step)
                    elif key == ord('D'):
                        target_roll = min(self.max_target_angle, target_roll + self.angle_step)
                    elif key == ord('R'):
                        target_roll = 0.0
                        target_pitch = 0.0
                    elif key == ord('Q'):
                        thrust_adj += 5.0
                    elif key == ord('E'):
                        thrust_adj -= 5.0
                    
                    key = self.keyboard.getKey()

                # Read current orientation
                rpy = self.imu.getRollPitchYaw()
                current_roll, current_pitch = rpy[0], rpy[1]

                # Calculate errors
                roll_err = target_roll - current_roll
                pitch_err = target_pitch - current_pitch

                # Proportional adjustments (Both inverted to ensure negative feedback)
                roll_adj = -roll_err * self.kp_angle
                pitch_adj = -pitch_err * self.kp_angle

                u = self.base_omega + thrust_adj
                m1 = u + pitch_adj + roll_adj
                m2 = u - pitch_adj - roll_adj
                m3 = u + pitch_adj - roll_adj
                m4 = u - pitch_adj + roll_adj

                cmds = [m1, m2, m3, m4]
                for i, m in enumerate(self.motors):
                    m.setVelocity(max(0.0, min(self.max_velocity, cmds[i])))

                # --- DUAL-AXIS DEBUG PRINTING ---
                has_active_error = abs(pitch_err) > math.radians(0.5) or abs(roll_err) > math.radians(0.5)
                
                if has_active_error and (step_count % 10 == 0):
                    print(
                        f"[DEBUG] P_Tgt: {math.degrees(target_pitch):+4.1f}° | "
                        f"P_Cur: {math.degrees(current_pitch):+4.1f}° | "
                        f"R_Tgt: {math.degrees(target_roll):+4.1f}° | "
                        f"R_Cur: {math.degrees(current_roll):+4.1f}° | "
                        f"Cmds: [{cmds[0]:.1f}, {cmds[1]:.1f}, {cmds[2]:.1f}, {cmds[3]:.1f}]"
                    )

                # Sensor reading & logging
                sim_time = float(self.robot.getTime())
                acc = self.accel.getValues()
                gyro = self.gyro.getValues()

                writer.writerow([
                    sim_time,
                    acc[0], acc[1], acc[2],
                    gyro[0], gyro[1], gyro[2],
                    rpy[0], rpy[1], rpy[2]
                ])

        print(f"[Logger] File closed. Saved to {self.log_path}")


if __name__ == "__main__":
    bench = StandaloneBench()
    bench.run()