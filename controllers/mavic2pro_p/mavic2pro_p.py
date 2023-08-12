from controller import Keyboard, Robot
from math import pi, pow
import sys
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def CLAMP(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic (Robot):
    # Constants, empirically found.
    k_vertical_thrust = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    k_vertical_offset = 0.6
    k_vertical_p = 3.0        # P constant of the vertical PID.
    k_roll_p = 50.0           # P constant of the roll PID.
    k_pitch_p = 30.0          # P constant of the pitch PID.

##    MAX_YAW_DISTURBANCE = 0.4
##    MAX_PITCH_DISTURBANCE = -1

    # Precision between the target position and the robot position in meters
    target_altitude = 5   

    def __init__(self):
        Robot.__init__(self)

        self.TIME_STEP = 32
        
        self.keyboard = Keyboard()
        self.keyboard.enable(self.TIME_STEP)

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.TIME_STEP)

        self.compass = self.getDevice("compass")
        self.compass.enable(self.TIME_STEP)

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.TIME_STEP)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.TIME_STEP)

        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.TIME_STEP)
        
        self.front_left_led = self.getDevice("front left led")
        self.front_right_led = self.getDevice("front right led")  

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")

        self.camera_roll_motor = self.getDevice("camera roll")
        self.camera_pitch_motor = self.getDevice("camera pitch")
#        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)
            
        print("Start the drone...\n");
    
        #Wait one second.
        while self.step(self.TIME_STEP) != -1:
            if self.getTime() > 1:
                break

    def run_1(self,pitch_disturbance,yaw_disturbance,roll_disturbance,target_altitude):
        key = self.keyboard.getKey()
        
        # print("You can control the drone with your computer keyboard:\n");
        # print("- 'up': move forward.\n");
        # print("- 'down': move backward.\n");
        # print("- 'right': turn right.\n");
        # print("- 'left': turn left.\n");
        # print("- 'shift + up': increase the target altitude.\n");
        # print("- 'shift + down': decrease the target altitude.\n");
        # print("- 'shift + right': strafe right.\n");
        # print("- 'shift + left': strafe left.\n");
        
        while key > 0:
            if key == Keyboard.UP:
                self.pitch_disturbance -= 2
                print("Hello UP")
                break
            elif key == Keyboard.DOWN:
                self.pitch_disturbance += 2
                print("Hello Down")
                break
            elif key == Keyboard.RIGHT:
                self.yaw_disturbance -= 1.3
                print("Hello Right")
                break
            elif key == Keyboard.LEFT:
                self.yaw_disturbance += 1.3
                print("Hello Left")
            elif key == Keyboard.SHIFT_RIGHT:
                self.roll_disturbance -= 0.1
                print("Hello SHIFT RIGHT")
            elif key == Keyboard.SHIFT_LEFT:
                self.roll_disturbance += 0.1
                print("Hello SHIFT LEFT")
            elif key == Keyboard.SHIFT_UP:
                self.target_altitude += 0.05
                print("Hello SHIFT UP")
            elif key == Keyboard.SHIFT_DOWN:
                self.target_altitude -= 0.05
                print("Hello SHIFT DOWN")

            key = self.keyboard.getKey()
        return self.pitch_disturbance, self.yaw_disturbance, self.roll_disturbance, self.target_altitude

    def run(self):
        t1 = self.getTime()
        self.roll_disturbance = 0
        self.pitch_disturbance = 0
        self.yaw_disturbance = 0
        
        while self.step(self.TIME_STEP) != -1:        
            time = self.getTime()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            # self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # roll = self.imu.getRollPitchYaw()[0]
            # pitch = self.imu.getRollPitchYaw()[1]
            
            # altitude = self.gps.getValues()[1]
            
            # roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            
            led_state: bool = int(time) % 2
            
            self.front_left_led.set(led_state)
            self.front_right_led.set(not led_state)
            
            # self.camera_roll_motor.setPosition(-0.115 * roll_acceleration)
            # self.camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)
            if altitude > self.target_altitude - 1:
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    self.pitch_disturbance, self.yaw_disturbance, self.roll_disturbance, self.target_altitude = self.run_1(
                        self.pitch_disturbance, self.yaw_disturbance, self.roll_disturbance, self.target_altitude)
                    t1 = self.getTime()

                pitch_input = (self.k_pitch_p * CLAMP(pitch, -1, 1)) + pitch_acceleration + self.pitch_disturbance

                roll_input = (self.k_roll_p * CLAMP(roll, -1, 1)) + roll_acceleration + self.roll_disturbance

                clamped_difference_altitude = CLAMP(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0);
                vertical_input = self.k_vertical_p * pow(clamped_difference_altitude, 3.0)

                yaw_input = self.yaw_disturbance
                
                front_left_motor_input = self.k_vertical_thrust + vertical_input - yaw_input + pitch_input - roll_input
                front_right_motor_input = self.k_vertical_thrust + vertical_input + yaw_input + pitch_input + roll_input
                rear_left_motor_input = self.k_vertical_thrust + vertical_input + yaw_input - pitch_input - roll_input
                rear_right_motor_input = self.k_vertical_thrust + vertical_input - yaw_input - pitch_input + roll_input
                
                print(self.gps.getValues())
    
                self.front_left_motor.setVelocity(front_left_motor_input)
                self.front_right_motor.setVelocity(-front_right_motor_input)
                self.rear_left_motor.setVelocity(-rear_left_motor_input)
                self.rear_right_motor.setVelocity(rear_right_motor_input)
    
        # return True

# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()