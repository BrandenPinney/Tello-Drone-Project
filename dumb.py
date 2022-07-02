from djitellopy import Tello
from time import sleep

drone = Tello()
drone.connect()
drone.streamon()

for i in range(8):

	drone.takeoff()
	sleep(10)
	drone.land()
	sleep(10)
