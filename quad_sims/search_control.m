function u = search_control(t,target,x,start)
% assume target 1 is the pixels to the right and 2 is pixels up
% x is measured state (noisy and incomplete, assume p q r, horizontal frame
% speed components, x,y,z positon available

% Differentiators are probably unworkable in noisy data sets
%     -maybe could be fixed with an estimator (try to avoid though)

% is this the first time step?
if nargin == 4 || isempty(start)
    start = false;
end

persistent x_diff y_diff z_diff x_diff_int y_diff_int z_diff_int step_time

if start || isempty(x_diff)
    x_diff = 0;
end

if start || isempty(y_diff)
    y_diff = 0;
end

if start || isempty(z_diff)
    z_diff = 0;
end

if start || isempty(x_diff_int)
    x_diff_int = 0;
end

if start || isempty(y_diff_int)
    y_diff_int = 0;
end

if start || isempty(z_diff_int)
    z_diff_int = 0;
end

% time step
if start || isempty(step_time)
    dt = 0;
    step_time = t;
else
    step_time_m1 = step_time;
    step_time = t;
    dt = step_time - step_time_m1;
end

global KP_alt KI_alt KD_alt KP_roll KI_roll KD_roll KP_pitch KI_pitch KD_pitch th0

% SATURATION LIMITS
roll_sat = 20*pi/180;
pitch_sat = 20*pi/180;
throttle_sat = 1;

% transforms
Rib = angle2dcm(x(9), x(8), x(7));
Rbi = Rib';
Rib_yaw = angle2dcm(x(9), 0, 0);
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

%proportional
x_diff_inertial = target(1) - x(1);
y_diff_inertial = target(2) - x(2);
z_diff_inertial = target(3) - x(3);
vec_diff = Rib_yaw * [x_diff_inertial;y_diff_inertial;z_diff_inertial];
x_diff =  vec_diff(1);
y_diff = vec_diff(2);
z_diff = vec_diff(3);

%derivative
velb = x(4:6)';
Rtb = angle2dcm(0, x(8), x(7));
Rbt = Rtb';
vel = Rbt*velb;


% ALTITUDE CONTROL
z_diff_prev = z_diff;

%integrator
z_diff_int = z_diff_int + z_diff*dt;

%derivative
if start
    z_diff_dot = 0;
else
    z_diff_dot = vel(3);
end

% throttle feed forward
throttle_com_ff = th0/(cos(x(7))*cos(x(8)));

%combine PID and FF
throttle_com = throttle_com_ff - KP_alt*z_diff - KI_alt*z_diff_int + KD_alt*z_diff_dot;

% throttle saturation
if throttle_com > throttle_sat
    throttle_com = throttle_sat;
elseif throttle_com < 0;
    throttle_com = 0;
end


% ROLL CONTROL
y_diff_prev = y_diff;

%integrator
y_diff_int = y_diff_int + y_diff*dt;

%derivative
if start
    y_diff_dot = 0;
else
    y_diff_dot = vel(2);
end

%combine PID and FF
roll_com = KP_roll*y_diff - KI_roll*y_diff_int - KD_roll*y_diff_dot;

% throttle saturation
if roll_com > roll_sat
    roll_com = roll_sat;
elseif roll_com < -roll_sat
    roll_com = -roll_sat;
end

% PITCH CONTROL
x_diff_prev = x_diff;

%integrator
x_diff_int = x_diff_int + x_diff*dt;

%derivative
if start
    x_diff_dot = 0;
else
    x_diff_dot = vel(1);
end

%combine PID and FF
pitch_com = -KP_pitch*x_diff + KI_pitch*x_diff_int + KD_pitch*x_diff_dot;

% throttle saturation
if pitch_com > pitch_sat
    pitch_com = pitch_sat;
elseif pitch_com < -pitch_sat
    pitch_com = -pitch_sat;
end

% COMMAND
u = [pitch_com roll_com x(end-1) throttle_com]; 
end


