import scipy as sp
import numpy as np
def error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,z,Pk=0,xk=0)
	#Measurement matrix
	H_til = [1 0];

	#State transition matrix
	F = [0 1;0 0];
	phi = sp.expm2(F*dt);
	#Time update
	if start:
		P_bar = P_bar_0
		x_bar = x_bar_0
	else:
		P_bar = np.dot(np.dot(phi,Pk),np.transpose(phi)) + Q
		x_bar = phi*xk_t
	#Measurement Update
	y               = z - np.dot(H_til,x_bar);
	K               = np.transpose(np.dot(P_bar,H_til))/(np.dot(np.dot(H_til,P_bar),H_til) + R);
	xk_plus       = x_bar + (K*y);
	Pk_plus       = (np.identity(2) - K*H_til)*P_bar;
	
	return xk_plus Pk_plus
