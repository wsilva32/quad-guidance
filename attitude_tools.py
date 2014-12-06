import numpy as np
from math import *

def angle2dcm(yaw,roll,pitch): #calculate direction cosine matrix
	Rx = np.array([[1, 0, 0],[0, cos(roll), -sin (roll)], [0, sin(roll), cos(roll)]])
	Ry = np.array([[cos(pitch), 0 , sin (pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
	Rz = np.array([[cos(yaw), -sin (yaw), 0], [sin(yaw), cos(yaw), 0],[0, 0, 1]])
	Rxy = np.dot(Rx,Ry)
	R = np.dot(Rxy,Rz)
	return R 

