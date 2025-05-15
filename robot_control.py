from robot_controller import RobotController
from differential_drive import DiffDriveRobot
import time
from robot_navigation import Navigation

# PIN FORMAT: ({left_PWM}, {right_PWM}, {left_Direction}, {right_Direction})
PINS = (12, 13, 4, 17)
# PIN FORMAT: ({Encoder_l_A}, {Encoder_l_B}, {Encoder_r_A}, {Encoder_r_B})
ENCODER_PINS = (5, 6, 27, 22)

DT = 0.01
WHEEL_RADIUS = 1
WHEEL_SEP = 1
K_P = 1
K_I = 0.25


X_GOAL = 2 # Place holder values
Y_GOAL = 2 # Place holder values

# Example set up - need to figure out what the actual parameters are
# Figures out a new duty_cycle needed to match desired velocity and updates motors accordingly
controller = RobotController(Kp=K_P, Ki=K_I, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEP, PWM_pins=PINS) 
# Solves for general kinematic variables and updates known parameters like the actual angular velocities from rotary encoder, used to feedback into the controller
robot = DiffDriveRobot(wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEP, encoder_PINs=ENCODER_PINS, _dt=DT)
navigator = Navigation(x_goal=X_GOAL, y_goal=Y_GOAL, robot=robot)

poses = []
velocities = []
duty_cycle_commands = []
while(True):
    
    time.sleep(DT)

    # Navigate
    navigator.navigate()

    # Example motion using controller   
    controller.drive(1, 1, robot.wl, robot.wr) # Where robot.wr is the measured wheel vel
    # Simulate robot motion - send duty cycle command to robot
    x, y, th = robot.update_odometry() # Update kinematic parameters based on current angular speed
    # Log data
    poses.append([x,y,th])
    velocities.append([robot.wl,robot.wr])