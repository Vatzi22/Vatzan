#####TRIAL

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math

def arm_and_takeoff(targetHeight):

	while vehicle.is_armable!=True:
		print("Waiting for vehicle")
		time.sleep(1)
	print("Vehicle is now ready")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Armed")

	vehicle.simple_takeoff(targetHeight)

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")

	return None

vehicle = connect('127.0.0.1:14550',wait_ready=True)

arm_and_takeoff(5)
print("Vehicle reached target altitude")


vehicle.mode=VehicleMode('LAND')
while vehicle.mode!='LAND':
	print("Waiting for drone to enter LAND mode")
	time.sleep(1)
print("Vehicle now landing")

