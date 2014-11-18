function u = quad_control(t,target,x,speed_des,start)
% assume target 1 is the pixels to the right and 2 is pixels up
% x is measured state (noisy and incomplete, assume p q r, horizontal frame
% speed components, x,y,z positon available

% Differentiators are probably unworkable in noisy data sets
%     -maybe could be fixed with an estimator (try to avoid though)

% is this the first time step?
if nargin == 4 || isempty(start)
    start = false;
end
global FoVv FoVh FoVpv FoVph T_max m g
persistent gam_d gam_int speed_err speed_err_int yaw_diff yaw_diff_int sw_slip sw_slip_int step_time


if start || isempty(gam_d)
    gam_d = 0;
end

if start || isempty(gam_int)
    gam_int = 0;
end

if start || isempty(speed_err)
    speed_err = 0;
end

if start || isempty(speed_err_int)
    speed_err_int = 0;
end

if start || isempty(yaw_diff)
    yaw_diff = 0;
end

if start || isempty(yaw_diff_int)
    yaw_diff_int = 0;
end

if start || isempty(sw_slip)
    sw_slip = 0;
end

if start || isempty(sw_slip_int)
    sw_slip_int = 0;
end

if isnan(target(1)) || isnan(target(2))
    ME = MException('VerifyTarget:TargetAcquired', ...
        'Target Lost');
    throw(ME);
end
% previous error for differentiators
gam_d_m1 = gam_d;
speed_err_m1 = speed_err;
yaw_diff_m1 = yaw_diff;
sw_slip_m1 = sw_slip;

% time step
if start || isempty(step_time)
    dt = 0;
    step_time = t;
else
    step_time_m1 = step_time;
    step_time = t;
    dt = step_time - step_time_m1;
end

global KP_t KI_t KD_t KP_p KI_p KD_p KP_r KI_r KD_r KP_rl KI_rl KD_rl
% % GAINS
% 
% % throttle
% KP_t = 1;
% KI_t = 0;
% KD_t = 0;
% 
% % pitch
% KP_p = 0.05;
% KI_p = 0;
% KD_p = 0;
% 
% % body z rotation rate
% KP_r = 1;
% KI_r = 0;
% KD_r = 0;
% 
% % roll
% KP_rl = 1;
% KI_rl = 0;
% KD_rl = 0;

% SATURATION LIMITS

roll_sat = 20;
pitch_sat = 20;
r_sat = 2;
throttle_sat =1;

% CONSTANTS



% MEASUREMENT CONVERSION (target pixels to desired flight path and yaw difference)

% % field of view from center in degrees
% FoVh = 40*pi/180;
% FoVv = 40*pi/180;
% 
% % field of view in pixels from center
% FoVph = 320;
% FoVpv = 320;

% range to virutal plane in pixels
rangeh = FoVph/tan(FoVh);
rangev = FoVpv/tan(FoVv);


% z down
el = atan2(-target(2), rangev);
az = atan2(target(1), rangeh);

% vector to the virutal plane point
vecb = [1; tan(az); tan(el)];
% normalize
vecb = vecb./norm(vecb);

% transforms
Rib = angle2dcm(x(9), x(8), x(7));
Rbi = Rib';

% we are interested in relative yaw angle so the yaw can be assumed zero
Rib_image = angle2dcm(0, x(8), x(7));
Rbi_image = Rib_image';

vec = Rbi_image*vecb;

% desired flightpath angle (positive is below horizon)
gam_d = atan2(vec(3),vec(1));
% yaw difference (positve is to the right of the vehicle)
yaw_diff = atan2(vec(2),vec(1));

% the controller error quantities are the yaw_diff and gam_d

% CONTROLLER OVERVIEW

% in general we want to yaw toward the target and climb up to it
% a speed control is also needed for image tracking
% Proposed design
% 
% roll is set to prevent sideways movement (no sideways motion should be
% needed or helpful here) needs to be kept minimal though
% 
% should maybe also try simplified system with simply setting roll at zero
% 
% speed control loop acts on pitch angle
% 
% altitude hold uses throttle with feed forward term from known pitch, roll and
% weight (ie the global z component must counter gravity) 
% 
% rotation rate (r) is set with PID (D may not be possible) control to the
% yaw difference (yaw_diff) with the assumption that the pitch is small
% (may want to investigate a better method later)

% ALTITUDE CONTROL 

% differentiator
if start
    gam_dot = 0;
else
    gam_dot = (gam_d - gam_d_m1)/dt;
end

% integrator
gam_int = gam_int + gam_d*dt;

% throttle feed forward
% need
% cos(x(7))*cos(x(8))*T - m*g = 0
% T = m*g/cos(x(7))*cos(x(8));
throttle_com_ff = m*g/(cos(x(7))*cos(x(8))*T_max);

% we may need P and I control here 
throttle_com = throttle_com_ff - KP_t*gam_d - KI_t*gam_int - KD_t*gam_dot;

% throttle saturation
if throttle_com > throttle_sat
    throttle_com = throttle_sat;
elseif throttle_com < 0;
    throttle_com = 0;
end


% SPEED CONTROL

% speed_des = 2;

% given no knowledge of drag coefficients on the aircraft take the simple
% gain tuning approach with the concept that zero pitch gives zero speed in
% the unperturbed case

% forward speed from state
vel = Rbi_image*[x(4) x(5) x(6)]';
speed = vel(1);

speed_err = speed - speed_des;

% differentiator
if start
    speed_err_dot = 0;
else
    speed_err_dot = (speed_err - speed_err_m1)/dt;
end



% integrator
speed_err_int = speed_err_int + speed_err*dt;


% pitch down speeds you up
pitch_com = KP_p*speed_err + KI_p*speed_err_int + KD_p*speed_err_dot;


% pitch saturation
if pitch_com > pitch_sat
    pitch_com = pitch_sat;
elseif pitch_com < -pitch_sat;
    pitch_com = -pitch_sat;
end

% YAW CONTROL

% differentiator

if start
    yaw_diff_dot = 0;
else
    yaw_diff_dot = (yaw_diff - yaw_diff_m1)/dt;
end



% integrator
yaw_diff_int = yaw_diff_int + yaw_diff*dt;

% command
r_com = KP_r*yaw_diff + KI_r*yaw_diff_int + KD_r*yaw_diff_dot; 

% r saturation
if r_com > r_sat
    r_com = r_sat;
elseif r_com < -r_sat;
    r_com = -r_sat;
end


% ROLL CONTROL

% sideways motion
sw_slip = vel(2);

% differentiator

if start
    sw_slip_dot = 0;
else
    sw_slip_dot = (sw_slip - sw_slip_m1)/dt;
end


% integrator
sw_slip_int = sw_slip_int + sw_slip*dt;

% command (positive roll -> positive slip)
roll_com = -KP_rl*sw_slip -KI_rl*sw_slip_int -KD_rl*sw_slip_dot;

% r saturation
if roll_com > roll_sat
    roll_com = roll_sat;
elseif roll_com < -roll_sat;
    roll_com = -roll_sat;
end


% COMMAND
u = [pitch_com roll_com r_com throttle_com];
end


