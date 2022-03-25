from re import X
from djitellopy import Tello
import cv2 as cv
from time import sleep
import movement as mv
import haar_cascade as hc
import mission
import math
import matplotlib.pyplot as plt
from matplotlib.path import Path
import numpy as np
import matplotlib.patches as patches


fbRange = [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480 # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = [] # List containing the locations of found turbines
detected_object = 0 # A flag to determine if the camera detected an object in the previous 5 frames


def backForth(drone, location, flyZone, moveIncr):
    '''Takes the drone object, the drones current location, a fly zone to search in a
    back and forth motion and a distance in cm to move each time. Fly zone must be a rectangular
    area flyZone[0] = xmin, flyZone[1] = xmax, flyZone[2] = ymin, 
    flyZone[3] = ymax
    Returns new location.'''

    xMin = flyZone[0] #+ 20           #safety bounds
    yMin = flyZone[2] #+ 20
    xMax = flyZone[1] #- 20 
    yMax = flyZone[3] #- 20

    #maxMove = 30                #how much to move at a time;
    shortLen = 30               #how far to move on short edge;
    yNew = location[1] + moveIncr * math.sin(math.radians(location[2]))
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
            location = mv.move(location, drone, fwd=round(yMax-location[1]))
            location = mv.move(location, drone, ccw=90)
            sleep(0.5)
            xNew = location[0] + moveIncr * math.sin(math.radians(location[2]))
            if xNew < xMin:
                print("Went through whole area")
                location = mv.return_path(location, drone)
    else:
        if yNew < yMin:
            location = mv.move(location, drone, fwd=round(location[1]-yMin))
            location = mv.move(location, drone, cw=91)
            sleep(0.5)
            xNew = location[0] + moveIncr * math.sin(math.radians(location[2]))
            if xNew < xMin:
                print("Went through whole area")
                drone.land()
                location = mv.return_path(location, drone)
    location = mv.move(location, drone, fwd = moveIncr)

    return location

def approx_cell_decomp(drone, droneLocation, obstacleList, boundary):
    '''input: drone reference, current drone location, array of obstactles, array of boundary coordinates
    Decomposes the area into cells and returns waypoints that are in the safe to fly zone.'''
    boundary = np.append(boundary, [boundary[0]], 0)
    w = 30
    xmin = np.amin(boundary[:,0]) + (w/2)
    xmax = np.amax(boundary[:,0]) - (w/2)
    ymin = np.amin(boundary[:,1]) + (w/2)
    ymax = np.amax(boundary[:,1]) - (w/2)
    cells_x = np.arange(xmin, xmax, w)
    cells_y = np.arange(ymin, ymax, w)
    cells = []
    waypoints = []
    for x in cells_x:
        for y in cells_y:
            cells =  np.append(cells, [x,y])
    cells = cells.reshape(-1,2)
    fig, ax = plt.subplots()
    
    for c in cells:
        xmin = c[0] - w/2
        xmax = c[0] + w/2
        ymin = c[1] - w/2
        ymax = c[1] + w/2
        pts = [[xmin, ymin],[xmax,ymin],[xmax, ymax],[xmin, ymax]]
        p_cell = Path(pts)
        isInside = []
        for ob in obstacleList:
            codes = [Path.MOVETO]
            for i in range(len(ob.coords)-1):
                codes.append(Path.LINETO)
            codes.append(Path.CLOSEPOLY)
            ob.coords = np.append(ob.coords, [ob.coords[0]], 0)
            p_obst = Path(ob.coords, codes)
            patch = patches.PathPatch(p_obst, facecolor='blue')
            ax.add_patch(patch)
            isInside = np.append(isInside, p_obst.contains_points(pts))
        if not(bool(isInside.any())):
            waypoints = np.append(waypoints, c)
            waypoints = waypoints.reshape(-1,2)
    plt.plot(boundary[:,0], boundary[:,1], color='black', lw=3, label='boundary')
    plt.scatter(cells[:,0], cells[:,1], marker='o', color='red', label='cells containing objects')
    plt.scatter(waypoints[:,0], waypoints[:,1], marker='o', color='green', label='waypoints')
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, 0.95))
    plt.show()
    return waypoints
         
        
def testBF(location):
    '''Tests back and for search function given the current location of the drone.
        bounds = [-270,0, 0, 270] works well for the drone cage'''
    mission_list = [1, 1, 1, 1]
    turbine_list = ["WindTurbine_2"]
    bounds = [-275,0, 0, 275]
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
        location = backForth(drone,location,bounds,50)
        sleep(0.5)

class obstacle: 
    def __init__(self, name, coords): 
        self.name = name 
        self.coords = coords

 

if __name__ == "__main__":
    '''Tests cell decomposition'''
    drone = Tello()
    #obstacles = np.array([obstacle('wind_turbine1', [[50,50],[150,50],[100,100],[75,120],[50,100]]),
     #                       obstacle('wind_turbine1', [[150,200],[300,200],[300,300],[200,250]])])
    obstacles = np.array([obstacle('wind_turbine1', [[50,50],[30,130],[80,120],[120,80],[60,40]]),
                          obstacle('wind_turbine1', [[150,200],[300,175],[275,300],[200,250]]),
                          obstacle('wind_tubine3', [[270, 78], [250,20], [280,50]])])

    approx_cell_decomp(drone, location, obstacles, [[0,0],[0,305],[305,305],[305,0]])


#orient toward long edge
#straight check camera
#if reach boundary
    #take care of boundary turn
    #check camera after turn
    #if reach boundary short edge
        #land
    #else move forward 