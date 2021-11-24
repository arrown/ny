import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

print ("connecting to Rover")
rover = connect("/dev/ttyUSB0",baud = 57600, wait_ready = True, heartbeat_timeout=120)
print("Rover connected")
rover.groundspeed = 0.7
time.sleep(1)

def goto_position_target_local_ned(north, east, down):

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

rover.mode = VehicleMode("GUIDED")

while rover.mode!='GUIDED':
	print("Waiting for change mode")
	time.sleep(1)
print("Rover in GUIDED MODE")

goto_position_target_local_ned(1,0,0)
print("close vehicle")
rover.close()
