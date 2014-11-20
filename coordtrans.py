from numpy  import *

def lla2flat(lat,lon,alt,lat0,lon0,alt0): #convert postion at lat lon alt
	# to flat ENU coordinates centered at lat0 lon0 alt0
	#import numpy as np
	a = 6378137.0
	f = 1/298.257223563
	#b = a*(1-f)
	lat = lat*pi/180
	lon = lon*pi/180
	lat0 = lat0*pi/180
	lon0 = lon0*pi/180
	e = sqrt(f*(2-f))
	N = a/(sqrt(1-e**2*(sin(lat))**2))
	N0 = a/(sqrt(1-e**2*(sin(lat))**2))
	x_ec =(alt + N)*cos(lat)*cos(lon) 
	y_ec =(alt + N)*cos(lat)*sin(lon)
	z_ec =(alt + (1-e**2)*N)*sin(lat)
	x_ec0 =(alt0 + N0)*cos(lat0)*cos(lon0) 
	y_ec0 =(alt0 + N0)*cos(lat0)*sin(lon0)
	z_ec0 =(alt0 + (1-e**2)*N)*sin(lat0)
	X_ec = array([x_ec,y_ec,z_ec]).reshape(3,1)
	X0_ec = array([x_ec0,y_ec0,z_ec0]).reshape(3,1)
	Xp = X_ec - X0_ec
	Rte = array([[-sin(lon0), cos(lon0),0],
	             [-cos(lon0)*sin(lat0), -sin(lat0)*sin(lon0), cos(lat0)],
	            [cos(lat0)*cos(lon0), cos(lat0)*sin(lon0), sin(lat0)]])
	X_flat = dot(Rte,Xp)
	x = X_flat[0][0]
	y = X_flat[1][0]
	z = X_flat[2][0]
	return (x,y,z)

def lla2flatdumb(lat,lon,alt,lat0,lon0,alt0):
	re = 6378137;
	re_c = re*cos((pi/180)*absolute(lat0))
	x = (lon-lon0)*(re_c*pi)/180
	y = (lat-lat0)*(re*pi)/180
	z = alt-alt0
 	return (x,y,z)

def flat2lla(x,y,z,lat0,lon0,alt0): #convert x,y,z to lat, lon, alt
	re = 6378137;
	re_c = re*cos((pi/180)*absolute(lat0))
	lon = x*180/(pi*re_c) + lon0
	lat = y*180/(pi*re) + lat0
	alt = z + alt0
	return (lat,lon,alt)














