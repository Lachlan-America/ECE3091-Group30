from _typeshed import Self
from differential_drive import DiffDriveRobot
import numpy as np

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y



class Navigation:
    
    def __init__(self, x_goal, y_goal, robot:DiffDriveRobot):
        self.primary_goal = Point(x_goal, y_goal)# a place to save the main goal when it is 
        self.goal = self.primary_goal # stores current location being navigated towards
        self.v = 0
        self.w = 0
        self.robot = robot
        self.is_obj_ahead = 0
    

    def navigate(self): # Provides movement towards target

        if (self.robot.x > np.arctan((self.goal.y-self.robot.y)/(self.goal.x-self.robot.x))): # If robot is facing too far to the left, turn right
            w = 1
            v= 0
        elif (self.robot.x > np.arctan((self.goal.y-self.robot.y)/(self.goal.x-self.robot.x))): # If robot is facing too far to the right, turn left
            w = -1
            v= 0
        elif(self.is_obj_ahead): # is_obj_ahead is a flag that will be informed by ultrasonic sensors to determine if object is ahead.
            self.obj_avoid()
        else: 
            if (self.robot.x != self.goal.x and self.robot.y != self.goal.y): # If robot is alligned with target, move towards it
                w = 0
                v = 1
            else: # if we are at the current goal
                if self.goal != self.primary_goal: # checks if we are at the primary goal
                    self.goal = self.primary_goal # if we're not at the primary goal (and instead are at the secondary goal), load the primary goal
                else:
                    print("niice") # woot
                
        
    # Super simple object avoidance.
    def objec_avoid(self): # This function is called if there is an obstacle detected in front of us. It sets a new goal 30 cm away to the left in order to avoid the obstacle
        AVOID_DIST = 0.3 # (metres) Distance to move before returning to orig
        TURN_ANGLE = np.pi/2 # angle we turn before moving
        self.goal.x = np.cos((self.robot.th + TURN_ANGLE)) * AVOID_DIST # sets new x 30 cm away 90 degrees from current facing direction
        self.goal.y = np.sin((self.robot.th + TURN_ANGLE)) * AVOID_DIST # sets new y 30 cm away 90 degrees from current facing direction


# currently we are treating the input from the ultrasonic as a binary thing. in front or not infront. I think it may be best to use the actual distance. 
# This would provide a lot more options for object avoidance
# e.g:
    # Turn 5 degrees
    # Check if there is an obstacle present at OBJ_AVOID_DIST/cos(5 degrees)
    # if so turn another 5 degrees.
    # check if there is an obstacle present at OBJ_AVOID_DIST/cos(10 degrees)
    # Keep going until no object is detected
    # turn an additional 5 degrees just in case
    # go forward AVOID_DIST distance