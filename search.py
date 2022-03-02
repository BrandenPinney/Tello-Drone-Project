from djitellopy import Tello
import cv2 as cv
from time import sleep
import movement as mv
import haar_cascade as hc
import mission
import math


fbRange = [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480 # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = [] # List containing the locations of found turbines
detected_object = 0 # A flag to determine if the camera detected an object in the previous 5 frames


def search(drone, location, flyZone):
    '''Takes the drone object, the drones current location and a fly zone to search 
    for a nearby object. Currently fly zone must be a square area.
    flyZone[0] = xmin, flyZone[1] = xmax, flyZone[3] = ymin, flyZone[4] = ymax
    Returns new location.'''

    xMin = flyZone[0] + 20           #safety bounds
    yMin = flyZone[2] + 20
    xMax = flyZone[1] - 20 
    yMax = flyZone[3] - 20

    maxMove = 30                #how much to move at a time;
    shortLen = 30               #how far to move on short edge;
    yNew = location[1] + maxMove * math.sin(math.radians(location[2]))
    print(location[0], location[1], location[2])
 
    #straighten out with horizontal closest to angle  <180=90 (left)       else=270 (right)
    if location[2]<90:
        location = mv.move(location, drone, ccw=90-location[2])
    elif location[2]<180:
        location = mv.move(location, drone, cw=location[2]-90)
    elif location[2]<270:
        location = mv.move(location, drone, ccw=270-location[2])
    else: 
        location = mv.move(location, drone, cw=location[2]-270)

    if location[2]<180:
        if yNew > yMax:
            location = mv.move(location, drone, ccw=90)
            sleep(0.5)
            xNew = location[0] + maxMove * math.sin(math.radians(location[2]))
            if xNew < xMin:
                print("Went through whole area")
                drone.land()
                #location = mv.move(location, drone, ccw = 90.5)
        location = mv.move(location, drone, fwd=maxMove)
    else:
        if yNew < yMin:
            location = mv.move(location, drone, cw=91)
            sleep(0.5)
            xNew = location[0] + maxMove * math.sin(math.radians(location[2]))
            if xNew < xMin:
                print("Went through whole area")
                drone.land()
                #location = mv.move(location, drone, cw = 90)
        location = location = mv.move(location, drone, fwd = maxMove)
    return location
            
        
if __name__ == "__main__":      #Tests search
    mission_list = [1, 1, 1, 1]
    turbine_list = ["WindTurbine_2"]
    bounds = [-100, 100, -20, 100]
    drone = Tello()
    drone.connect()
    sleep(0.5)
    print("Current battery remaining: ", drone.get_battery())
    sleep(0.3)
    drone.streamon()
    sleep(0.5)
    drone.takeoff()
    sleep(0.5)

    while True:
        location = search(drone,location,bounds)
        sleep(0.5)