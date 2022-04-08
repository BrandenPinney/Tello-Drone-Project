'''This file helps debug and simulate drone movement without needing to be connected to the drone'''
from djitellopy import Tello
import math
import random
from time import sleep


def move(new_location, drone, fwd = 0, back = 0, ccw = 0, cw = 0, up = 0, down = 0):
    '''Takes a list holding the x, y cartesian coordinates of the drone and the angle relative to takeoff [x, y, angle] (initialized as [0, 0, 0]).
    The variable for the drone being used is also required.
    Then commands for fwd and back will be taken that can range from 20-500, and ccw and cw from 0-359.
    The function will then rotate the drone first with counter-clockwise (ccw) being the priority, then move with forward (fwd) being the priority.
    The function will return a list with length 3 of updated coordinates and the drone angle as [x, y, angle]'''
    if up != 0:
        #drone.move_up(up)
        new_location[3] += up
    elif up == 0 and down != 0:
        if new_location[3] > down:
            #drone.move_down(down)
            new_location[3] -= down
    if ccw != 0:
        #drone.rotate_counter_clockwise(ccw)
        # Returned angle if ccw
        new_location[2] = (new_location[2] + ccw) % 360 
    elif ccw == 0 and cw != 0:
        #drone.rotate_clockwise(cw)
        if cw > new_location[2]:
            # Returned angle if cw angle is greater than the current angle
            new_location[2] = 360 - abs((new_location[2] - cw)) 
        else:
            # Returned angle if cw angle is less than the current angle
            new_location[2] = abs((new_location[2] - cw)) 
    if fwd != 0:
        # forward takes priority -- returns x and y coordinates after movement
        #drone.move_forward(fwd)
        new_location[0] += fwd * math.cos(math.radians(new_location[2]))
        new_location[1] += fwd * math.sin(math.radians(new_location[2]))
    elif fwd == 0 and back != 0:
        # returns x and y coordinates after backwards movement
        #drone.move_back(back)
        new_location[0] += -(back) * math.cos(math.radians(new_location[2]))
        new_location[1] += -(back) * math.sin(math.radians(new_location[2]))
    return new_location

def return_path(new_location, drone):
    '''Takes a list with length 3 of the current x, y coordinates and angle relative to takeoff of the drone [x, y, angle].
    The variable for the drone being used is also required.
    This function will first angle the drone back to zero degrees, rotate towards the origin, then move forward the calculated distance'''
    x = new_location[0]
    y = new_location[1]
    angle = new_location[2]
    # Rotate the drone to zero degrees
    
    try:
        return_angle = abs(math.degrees(math.atan(x/y)))
    except ZeroDivisionError:
        pass
    # cases for rotation based on current cartesian quadrant of the drone
     
    # calculate return vector distance and break it up into pieces if over 500 centimeters or land if under 20 centimeters.
    return_distance = int(math.sqrt(x**2 + y**2))
    while (return_distance != 0):
        if (return_distance >= 500):
            #drone.move_forward(500)
            return_distance -= 500
        elif (return_distance < 500) and (return_distance < 20):
            return_distance = 0
        else:
            #drone.move_forward(return_distance)
            
            return_distance = 0
    #drone.land()


