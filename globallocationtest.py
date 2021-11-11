import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
print "connecting to Drone"
Drone = connect("/dev/serial0", baudrate = 57600, wait_ready = True) # telemetry line to raspberry pi
Drone.airspeed = 1
Drone.groundspeed = 0.3
time.sleep(1)

print "connecting to Rover"
Rover = connect("/dev/", baudrate = 57600, wait_ready = True) #telemetry receiver
Rover.groundspeed = 1
time.sleep(1)

def arm_and_takeoff(aTargetAltitude):

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not Drone.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    Drone.mode    = VehicleMode("GUIDED")
    Drone.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not Drone.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    Drone.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", Drone.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if Drone.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def goto_position_target_local_ned_drone(north, east, down):

    msg = Drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    Drone.send_mavlink(msg)

def goto(dNorth, dEast, gotoFunction=Rover.simple_goto):

    currentLocation = Rover.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while Rover.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(Rover.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.25: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)
        
arm_and_takeoff(4)
goto(10,12)
# swarm code
lat = Drone.location.global_relative_frame.lat # drone's latitude
lon = Drone.location.global_relative_frame.lon # drone's longitude
nlat = Rover.location.global_relative_frame.lat # rover's latitude
nlon = Rover.location.global_relative_frame.lon # rover's longitude
dnorth = (nlat-lat)*rad*math.pi/180
deast = math.pi/180*(nlon-lon)*rad*math.cos(lat/180*math.pi)
goto_position_target_local_ned_drone(dnorth,deast,4)
time.sleep(3)
goto(-7,-13)
# swarm code
lat = Drone.location.global_relative_frame.lat # drone's latitude
lon = Drone.location.global_relative_frame.lon # drone's longitude
nlat = Rover.location.global_relative_frame.lat # rover's latitude
nlon = Rover.location.global_relative_frame.lon # rover's longitude
dnorth = (nlat-lat)*rad*math.pi/180
deast = math.pi/180*(nlon-lon)*rad*math.cos(lat/180*math.pi)
goto_position_target_local_ned_drone(dnorth,deast,4)
time.sleep(2)
drone.mode = VehicleMode("LAND")
while Drone.mode!='LAND':
    time.sleep(1)
    print("Waiting for drone to land")
print("Landing..")
print("close drone")
Drone.close()
print("close rover")
Rover.close()
