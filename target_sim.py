import numpy as np
from math import *
from attitude_tools import angle2dcm

def target_sim(t_pos,x,y,z,roll,pitch,yaww,FoVv, FoVh, FoVpv, FoVph)

#if size(t_pos,1) == 1
#    t_pos = t_pos'
#end

#target positon must be a column vector
#FoV and FoVh should be in radians
#FoVph and FoVpv should be in pixels 


# vector to target

vec = t_pos - np.array([[x],[y],[z]]));

# transforms
Rib = angle2dcm(yaw, pitch, roll)
Rbi = np.transpose(Rib)

# vector relative to aircraft
vec_b = np.dot(Rib,vec)

az = np.arctan2(vec_b[1][0],vec_b[0][0])
# elevation is positive up here
el = np.arctan2(-vec_b[2][0],vec_b[0][0])

if abs(az) > FoVh or abs(el) > FoVv:
    target = np.array([np.nan, np.nan])
else:

    # range to virutal plane in pixels
    rangeh = FoVph/np.tan(FoVh)
    rangev = FoVpv/np.tan(FoVv)
    
    target = np.array([rangeh*tan(az), rangev*tan(el)])
return target
    



