from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
from math import radians, cos, sin, sqrt, atan2,pi
import threading

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:8888')
args = parser.parse_args()

vehicles = []

# Connect to all vehicles
for i in range(4):
    vehicle = connect(args.connect, baud=115200, wait_ready=True)
    vehicles.append(vehicle)

leader_vehicle =vehicles[0]

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Is the vehicle armable? ", vehicle.is_armable)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Is the vehicle armed? ", vehicle.armed)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Define function to calculate distance between two GPS coordinates
def get_distance_metres(location1, location2):
    dlat = radians(location2.lat - location1.lat)
    dlong = radians(location2.lon - location1.lon)
    a = sin(dlat/2) * sin(dlat/2) + cos(radians(location1.lat)) * cos(radians(location2.lat)) * sin(dlong/2) * sin(dlong/2)
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return 6371000 * c

#Define function to get valid coordinates based on waypoint
def get_valid_coordinates(waypoint):
    try:
        # Ensure waypoint is a tuple of length 3
        if not isinstance(waypoint, tuple) or len(waypoint) != 3:
            raise ValueError("Invalid waypoint. It should be a tuple of three values.")

        # Check if all elements in the tuple are numeric
        if not all(isinstance(coord, (int, float)) for coord in waypoint):
            raise ValueError("Invalid waypoint. All values in the tuple should be numeric.")

        # Return waypoint directly
        return waypoint

    except ValueError as e:
        print("Error:", e)
        return None

# Inputs:
#   leader_coord: 1x3 vector (latitude, longitude, altitude) of the leader drone
#   side_length: side length of the square (in meters) ---- hard coded
# Outputs:
#   drone_coords: 4x3 matrix containing coordinates of the four drones
#    drone 1(leader_coord)       drone 2
#    drone 3                     drone 4
def calculate_drone_coordinates(leader_coord):
    # Earth radius in meters
    R = 6371000
    # side length of the square formation
    side_length = 5

    # Extract leader coordinates
    latc_leader, lonc_leader, altc_leader = leader_coord

    # Convert degrees to radians
    lat_leader = latc_leader * pi / 180.0
    lon_leader = lonc_leader * pi / 180.0

    # Calculate offset
    offset = side_length / sqrt(2.0)

    # Change in longitude and latitude
    lon_offset = offset / (R * cos(lat_leader))
    lat_offset = offset / R

    # Calculate coordinates of each drone
    drone_coords = []

    # Leader Drone (Drone 1)
    drone_coords.append((latc_leader, lonc_leader, altc_leader))
    # Drone 2
    drone_coords.append((latc_leader + lat_offset * 180.0 / pi, lonc_leader, altc_leader))
    # Drone 3
    drone_coords.append((latc_leader, lonc_leader + lon_offset * 180.0 / pi, altc_leader))
    # Drone 4
    drone_coords.append((latc_leader + lat_offset * 180.0 /pi, lonc_leader + lon_offset * 180.0 / pi, altc_leader))

    return drone_coords

# Function to execute path planning
def execute_path_planning(vehicle, waypoints):
    # Initialize the takeoff sequence to 5m
    arm_and_takeoff(vehicle, 5)

    for waypoint in waypoints:
        target_location = LocationGlobalRelative(*waypoint)
        vehicle.simple_goto(target_location)
        while True:
            remaining_distance = get_distance_metres(vehicle.location.global_relative_frame, target_location)
            print("Distance to waypoint for Drone: ", remaining_distance)
            if remaining_distance <= 1:
                print("Reached waypoint")
                time.sleep(2)  # Give the drone some time to stabilize at the waypoint
                break
            time.sleep(1)

    # Hover for 10 seconds
    time.sleep(10)

    print("Now let's land")
    vehicle.mode = VehicleMode("LAND")
    # Close vehicle object
    vehicle.close()



# Function to generate drone coordinates
def generate_coordinates(waypoint):
    # Get valid coordinates
    leader_coord = get_valid_coordinates(waypoint)
    if leader_coord is None:
        return None
    
    # Calculate drone coordinates
    drone_coords = calculate_drone_coordinates(leader_coord)
    return drone_coords

'''
# Fly to waypoints
print("Flying to waypoints:", waypoints)



for waypoint in waypoints:
    target_location = LocationGlobalRelative(*waypoint)
    vehicle.simple_goto(target_location)
    while True:
        remaining_distance = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        print("Distance to waypoint for Drone: ", remaining_distance)
        if remaining_distance <= 1:
            print("Reached waypoint")
            time.sleep(2)  # Give the drone some time to stabilize at the waypoint
            break
        time.sleep(1)

# Hover for 10 seconds
time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")
'''

# Get current drone's location
current_location = leader_vehicle.location.global_relative_frame

# Calculate waypoints relative to the current location
waypoints = [
    (current_location.lat + 5 * 1e-5, current_location.lon, current_location.alt),
    (current_location.lat, current_location.lon + 5 * 1e-5, current_location.alt),
    (current_location.lat - 5 * 1e-5, current_location.lon, current_location.alt),
    (current_location.lat, current_location.lon - 5 * 1e-5, current_location.alt)
]

# Generate coordinates for each waypoint
drone_coords = [generate_coordinates(waypoint) for waypoint in waypoints]


# Execute path planning for the leader drone
leader_thread = threading.Thread(target=execute_path_planning, args=(leader_vehicle, drone_coords[0]))
leader_thread.start()

# Execute path planning for the rest of the drones
threads = []
for vehicle, coords in zip(vehicles, drone_coords[1:]):
    thread = threading.Thread(target=execute_path_planning, args=(vehicle, coords))
    thread.start()
    threads.append(thread)

# Wait for all threads to finish
leader_thread.join()
for thread in threads:
    thread.join()

print("Formation flying complete")
