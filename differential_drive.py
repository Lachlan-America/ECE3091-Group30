import numpy as np
import gpiozero
# from IPython import display
from matplotlib import pyplot as plt
from time import time

MAX_STEPS = 100000
INCREMENTS_PER_PLATE = 4
PLATES_PER_ROTATION = 32
GEARS_RATIO = 40.1

DIVISION_RATIO = INCREMENTS_PER_PLATE / PLATES_PER_ROTATION / GEARS_RATIO 

class DiffDriveRobot:
    """
    Obtains current angular velocity of the wheels via the rotary encoders and updates odometry accordingly
    """  

    def __init__(self, wheel_radius, wheel_sep, encoder_PINs, _dt):
        
        self.x = 0.0            # Current x-position
        self.y = 0.0            # Current y-position 
        self.th = 0.0           # Current orientation in regards to the angle THETA
        
        self.wl = 0.0           # Rotational velocity left-wheel
        self.wr = 0.0           # Rotational velocity right-wheel
        
        self.dt = _dt

        self.r = wheel_radius       # The wheels' radius
        self.l = wheel_sep          # The wheel's axle length or separation
        
        self.prev_count_l = 0          # The current count of gaps that have been iterated over via the rotor encoder
        self.prev_count_r = 0          # The current count of gaps that have been iterated over via the rotor encoder

        self.rot_encoder_l = gpiozero.RotaryEncoder(encoder_PINs[0], encoder_PINs[1], MAX_STEPS)
        self.rot_encoder_r = gpiozero.RotaryEncoder(encoder_PINs[2], encoder_PINs[3], MAX_STEPS)


    def obtain_angular_vel(self):
        """
        Used to determine actual speed of wheels: Output is rotational velocity of wheel used to keep track of position
        """

        delta_count_l = (self.rot_encoder_l.steps - self.prev_count_l)                 # The change in time since the last update
        delta_count_r = (self.rot_encoder_r.steps - self.prev_count_r)                 # The change in time since the last update

        count_speed_l = delta_count_l/self.dt    # found speed in terms of steps, need to find steps per rotation 
        count_speed_r = delta_count_r/self.dt 
                                                # self.pre_time is defined upon calling of class. Under current design, this will cause an issue on the very first call. 
                                                # should work fine when working continuously
        self.prev_count_l = self.rot_encoder_l.steps
        self.prev_count_r = self.rot_encoder_r.steps

        w_speed_l = count_speed_l / DIVISION_RATIO 
        w_speed_r = count_speed_r / DIVISION_RATIO 

        return w_speed_l, w_speed_r         # The current output angular velocity based on rotor encoder
    

    def calc_base_velocity(self, wl, wr):
        """
        The generalised velocity as a whole. In terms of the generalised velocity the robot moves along with its total rotation
        as the difference between each of the rotational velocities of the wheels.
        """

        v = (wl*self.r + wr*self.r)/2.0         # Total velocity of the robot
        w = (wl - wr)/self.l                    # Net rotational velocity due to the wheels
        
        return v, w             # Outputs generalised kinematic variables in order to update position
    

    def update_odometry(self):
        
        """
        Updates the kinematic variables based off what is read from the motor. Keeps track of our position in a virtualised world
        using odometry; uses motor values to gives us an estimate of our position relative to the initial position.
        """
        # TODO: Insert parameters for the pins 
        self.wl, self.wr = self.obtain_angular_vel()         # Obtains the right and left wheel angular velocities and the time interval
        
        v, w = self.calc_base_velocity(self.wl, self.wr)    # Obtain the general kinematic variables to update the odometry
        # TODO: Figure out a replacement for dt

        if(w == 0):
            self.x = self.x + self.dt*v*np.cos(self.th)         # Updates the x-coordinate based on the net velocity of the robot
            self.y = self.y + self.dt*v*np.sin(self.th)         # Updates the y-coordinate based on the net velocity of the robot
        else:
            th_new = self.th + w*self.dt                      
            self.x = self.x + v/w * (-np.sin(self.th) + np.sin(th_new))
            self.y = self.y + v/w * (np.cos(self.th) - np.cos(th_new))
            self.th = th_new                                            # Updates the facing-angle based on the net rotation of the robot
        

        return self.x, self.y, self.th          # Outputs parameters to be used for plotting, simulation, etc.
