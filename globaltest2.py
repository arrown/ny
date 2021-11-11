import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
print "connecting to Drone"
drone = connect("/dev/serial0", baudrate = 57600, wait_ready = True) # telemetry line to raspberry pi
drone.airspeed = 1
drone.groundspeed = 0.3
rover.parameters['WP_SPEED']=0.5
time.sleep(1)

print "connecting to Rover"
rover = connect("/dev/", baudrate = 57600, wait_ready = True) #telemetry receiver
rover.groundspeed = 1
time.sleep(1)

def arm_and_takeoff(aTargetAltitude):

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not drone.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    drone.mode    = VehicleMode("GUIDED")
    drone.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not drone.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    drone.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", drone.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if drone.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
        
def arming_rover():
    print ("Basic pre-arm checks")
    while not rover.is_armable:
        print("waiting for vehicle to initialise...")
        time.sleep(1)
    
    print ("Arming motors")
    rover.mode = VehicleMode("GUIDED")
    rover.armed = True
  
    while not rover.armed:
        print("waiting for arming...")
        time.sleep(1)
   
def get_location_metres(original_location, dNorth, dEast):

    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto_position_target_local_ned_drone(north, east, down):

    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    drone.send_mavlink(msg)

def goto_drone(targetLocation):
	distanceToTargetLocation = get_distance_metres(targetLocation,drone.location.global_relative_frame)

	drone.simple_goto(targetLocation)

	while drone.mode.name=="GUIDED":
		currentDistance = get_distance_metres(targetLocation,drone.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.05:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None

def goto_position_target_local_ned_rover(north, east, down):

    msg = rover.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    rover.send_mavlink(msg)

def goto_rover(dNorth, dEast, gotoFunction=rover.simple_goto):

    currentLocation = rover.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while rover.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(rover.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.25: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)

def make_gap(old,new):
	gap = new-(new-old)*0.1
	return gap
	
arm_and_takeoff(4)
arming_rover()
goto_rover(10,12)
time.sleep(20)
# swarm code
lat = make_gap(drone.location.global_relative_frame.lat,rover.location.global_relative_frame.lat)
lon = make_gap(drone.location.global_relative_frame.lon,rover.location.global_relative_frame.lon)
wp1 = LocationGlobalRelative(lat,lon,4)
goto_drone(wp1)
time.sleep(3)
goto_rover(-7,-13)
time.sleep(20)
# swarm code
lat = make_gap(drone.location.global_relative_frame.lat,rover.location.global_relative_frame.lat)
lon = make_gap(drone.location.global_relative_frame.lon,rover.location.global_relative_frame.lon)
wp1 = LocationGlobalRelative(lat,lon,4)
goto_drone(wp1)
time.sleep(2)
drone.mode = VehicleMode("LAND")
while drone.mode!='LAND':
    time.sleep(1)
    print("Waiting for drone to land")
print("Landing..")
print("close drone")
drone.close()
print("close rover")
rover.close()
