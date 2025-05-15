import numpy as np
import gpiozero
# from IPython import display
from matplotlib import pyplot as plt
from time import time

FREQUENCY = 4000

class RobotController:
    """
    Uses proportional feedback to adjust motor outputs to match 
    their desired angular velocities for the case of a differential drive
    """

    def __init__(self, Kp, Ki, wheel_radius, wheel_sep, PWM_pins):
        
        self.Kp = Kp                # A proportional controller error constant (has to be fine-tuned)
        self.Ki = Ki                # A proportional controller error constant (has to be fine-tuned)
        self.r = wheel_radius       
        self.l = wheel_sep          # The length of the wheel's axle or separation
        self.e_sum_l = 0            # The total error between desired and actual left wheel
        self.e_sum_r = 0            # The total error between desired and actual right wheel

        self.PWM_l = gpiozero.PWMOutputDevice(pin=PWM_pins[0], active_high=True, initial_value=0, frequency=FREQUENCY)          # Initialize PWM outputs
        self.PWM_r = gpiozero.PWMOutputDevice(pin=PWM_pins[1], active_high=True, initial_value=0, frequency=FREQUENCY)
        
        self.direction_l = gpiozero.OutputDevice(pin=PWM_pins[2], initial_value=1)                                                               # Initialize pins that determine motor directions
        self.direction_r = gpiozero.OutputDevice(pin=PWM_pins[3], initial_value=1)


    def prop_control(self, w_desired, w_measured, e_sum):
        """
        Takes in the measured rotational velocity and linear velocity 
        and gives out the duty-cycle and error needed to minimise error
        """

        duty_cycle = min(max(-1, self.Kp*(w_desired - w_measured) + self.Ki*e_sum), 1)          # The duty-cycle that should be achieved to minimize error      
        e_sum += (w_desired - w_measured)                                                       # A total amount of error

        direction = 1 # Currently in fwd direction 

        if(duty_cycle < 0):
            direction = 0

        return abs(duty_cycle), direction, e_sum
        
        
    def drive(self, v_desired, w_desired, wl, wr):
        """
        Takes in the rotational velocity and linear velocity and gives out the duty cycle and error
        """

        wl_desired = v_desired/self.r + self.l*w_desired/2                                       # Obtains the desired angular veloctiy from our desired velocity/angular velocity
        wr_desired = v_desired/self.r - self.l*w_desired/2
        
        duty_cycle_l, self.direction_l, self.e_sum_l = self.prop_control(wl_desired, wl, self.e_sum_l)            # Determines the duty_cycle needed for PWM for the desired angular velocities
        duty_cycle_r, self.direction_r, self.e_sum_r = self.prop_control(wr_desired, wr, self.e_sum_r)

        self.update_motors(duty_cycle_l, duty_cycle_r, self.direction_l, self.direction_r)                                          # Update the PWM duty-cycle needed based on proportional feedback


    def update_motors(self, duty_cycle_l, duty_cycle_r, direction_l, direction_r):
        """
        Used to update motors and PWM
        """
        self.PWM_l.value = duty_cycle_l     # TODO: Update motor PWM values to update motor speeds with newly found duty_cycle
        self.PWM_r.value = duty_cycle_r  

        self.direction_l.value = direction_l
        self.direction_r.value = direction_r


