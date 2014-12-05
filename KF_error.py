import scipy as sp
import numpy as np
from scipy import linalg
def error_derivative_KF(start,P_bar_0,x_bar_0,dt,Q,R,z,Pk=0,xk=0):
	#Measurement matrix
	H_til = np.array([1,0])

	#State transition matrix
	F = F = np.array([[0,1],[0,0]])
	phi = sp.linalg.expm(F*dt);
	#Time update
	if start:
		P_bar = P_bar_0
		x_bar = x_bar_0
	else:
		P_bar = np.dot(np.dot(phi,Pk),np.transpose(phi)) + Q
		x_bar = np.dot(phi, xk)
	#Measurement Update
	#print 
	y               = z - np.dot(H_til,x_bar)
	K               = np.dot(P_bar,H_til.reshape(2,1))/(np.dot(np.dot(H_til,P_bar),H_til.reshape(2,1)) + R)
	xk_plus       = x_bar + K*y
	#print np.dot(np.dot(K,np.array([H_til])),P_bar)
	Pk_plus       = np.identity(2) - np.dot(np.dot(K,np.array([H_til])),P_bar)
	
	return xk_plus,Pk_plus
