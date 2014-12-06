import scipy as sp
from scipy import linalg
import numpy as np
def velocity_KF(start,P_bar_0,x_bar_0,dt,Q,R,z,Pk=0,xk=0):
	#Measurement matrix
	H_til = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]])

	#State transition matrix
	F = np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]);
	phi = sp.linalg.expm(F*dt);
	#Time update
	if start:
		P_bar = P_bar_0
		x_bar = x_bar_0
	else:
		P_bar = np.dot(np.dot(phi,Pk),np.transpose(phi)) + Q
		x_bar = np.dot(phi, xk)
	#Measurement Update
	y               = z - np.dot(H_til,x_bar)
	K               = np.dot(np.dot(P_bar,np.transpose(H_til)),sp.linalg.inv(np.dot(np.dot(H_til,P_bar),np.transpose(H_til)) + R))
	xk_plus       = x_bar + np.dot(K,y)
	Pk_plus       = (np.identity(6) - np.dot(np.dot(K,H_til),P_bar))
	#print x_bar
	#print np.dot(K,y)
	#print y
	#print K
	return xk_plus,Pk_plus
