from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

print ("connecting to Drone")
drone = connect("/dev/ttyACM0", wait_ready = True, timeout = 120, heartbeat_timeout=120)
print ("Drone connected")
drone.airspeed = 0.3
drone.groundspeed = 0.3
time.sleep(1)

print ("connecting to Rover")
rover = connect("udp:127.0.0.1:14552",baud = 57600, wait_ready = True, timeout = 120, heartbeat_timeout=120)
print ("Rover connected")
rover.groundspeed = 0.7
time.sleep(1)

#------------------------------------------------------------

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

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)


#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 1
marker_size     = 20 #- [cm]
freq_send       = 1 #- Hz

land_alt_cm         = 100
angle_descend       = 20*deg_2_rad
land_speed_cms      = 30.0



#--- Get the camera calibration path

calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')                                    
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)
                
time_0 = time.time()
#----------------------Swarm Start --------------------------------------

altitude = 4
arm_and_takeoff(altitude)
time.sleep(4)
start = time.perf_counter()
while True:
    lat = rover.location.global_relative_frame.lat # rover's latitude
    lon = rover.location.global_relative_frame.lon # rover's longitude
    location = LocationGlobalRelative(lat, lon, altitude)
    drone.simple_goto(location)
    end = time.perf_counter()
    tmp = end-start
    if tmp >=15:
        a = input("continue?")
        if a == 'y':
            start = time.perf_counter()
            pass
        elif a == 'n':
            print("start landing")
            break
        else:
            start = time.perf_counter()
            pass
    time.sleep(2)
    
#------------------------Marker Start------------------------------------ 
while True:                

    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    if marker_found:
        x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
        uav_location        = drone.location.global_relative_frame
        
        #-- If high altitude, use baro rather than visual
        if uav_location.alt >= 5.0:
            print 
            z_cm = uav_location.alt*100.0
            
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

        
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            # print ""
            print " "
            print "Altitude = %.0fcm"%z_cm
            print "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg)
            
            north, east             = uav_to_ne(x_cm, y_cm, drone.attitude.yaw)
            print "Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, drone.attitude.yaw*rad_2_deg)
            
            marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
            #-- If angle is good, descend
            if check_angle_descend(angle_x, angle_y, angle_descend):
                print "Low error: descending"
                location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
            else:
                location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                
            drone.simple_goto(location_marker)
            print "UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon)
            print "Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon)
            
        #--- COmmand to land
        if z_cm <= land_alt_cm:
            if drone.mode == "GUIDED":
                print (" -->>COMMANDING TO LAND<<")
                drone.mode = "LAND"
