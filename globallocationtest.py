import math
import time

lon = 127.07463
lat = 37.55000
nlon = 127.075593
nlat = 37.550859
rad = 6378137
dnorth = (nlat-lat)*rad*math.pi/180
deast = math.pi/180*(nlon-lon)*rad*math.cos(lat/180*math.pi)
print dnorth,deast
