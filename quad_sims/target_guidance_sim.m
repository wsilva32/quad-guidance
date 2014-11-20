%% Simulation
% clear all

% INPUT AND STATE DESCRIPTION
% u = [pitch_com roll_com r_com throttle_com]'
% where r is the rate about body z (need to verify this is correct)
% x = [x y z u v w roll pitch yaw r throttle]

% TIME SEQUENCE

% time settings
tmax = 300;
rate = 100;

dt = 1/rate;
t = 0:dt:tmax;

% PARAMETERS

global FoVv FoVh FoVpv FoVph T_max m g CDA

% field of view from center in degrees
FoVh = 40*pi/180;
FoVv = 40*pi/180;

    
% field of view in pixels from center
FoVph = 320;
FoVpv = 320;

% gravity
g = 9.807;

% mass
m = 2;

% measurement noise

% turn on measurment noise
noise = true;

sig_pos = 0.1;
sig_vel = 0.1;
sig_euler = 1*pi/180;
sig_r = 0.2*pi/180;
sig_X = [sig_pos*ones(1,3) sig_vel*ones(1,3) sig_euler*ones(1,3) sig_r 0]; %no noise added to the throttle state because this is not measured 

% time delays (first order model)
global t_const_roll t_const_pitch t_const_yaw_rate t_const_throttle
% time constants
t_const_roll = 0.5;
t_const_pitch = 0.5;
t_const_yaw_rate = 0.3;
t_const_throttle = 0.15;


% wind disturbances

global sig_w_x sig_w_y sig_w_z Wd
sig_w_x = 0.15*dt;
sig_w_y = 0.15*dt;
sig_w_z = 0.15*dt;
Wd = 0;


% max thrust (in real system will be approximate as linear from thrust for
% level flight)
T_max = 50;

CDA = 1*0.4;

% target position
t_pos = [2 1 -0.75];

% desired vehicle speed
speed_des = 1;

% hover thrust
th0 = m*g/T_max;

% GAINS

global KP_t KI_t KD_t KP_p KI_p KD_p KP_r KI_r KD_r KP_rl KI_rl KD_rl

% throttle
KP_t = 0.35;
KI_t = 0;
KD_t = 0.3;

% pitch
KP_p = 0.55;
KI_p = 0.2;
KD_p = 0.08;

% body z rotation rate
KP_r = 4;
KI_r = 0;
KD_r = 1;

% roll
KP_rl = 2.3;
KI_rl = 0;
KD_rl = 0.2;

% SIMUALTION SETUP

% initial state (hover)
x0 = [0 0 0 0 -1.3 0 0 0 0 0 th0];

% loops
n = length(t);
l = length(x0);
% preallocate state
u = zeros(n,4);
x = zeros(n,length(x0));
x(1,:) = x0;

start = true;
end_ind = n;
for i = 1:n-1
    i
    target = target_sim(t_pos,x(i,:));
    try
        if noise
            x_meas = x(i,:) + (diag(sig_X.^2)*randn(l,1))';
        else
            x_meas = x(i,:);
        end
        u(i,:) = quad_control(t(i),target,x_meas,speed_des,start);
    catch
        warning('Target Lost')
        end_ind = i;
        break
    end
    tspan = [t(i) t(i+1)];
    [~,X] = ode45(@simple_quad_dyn,tspan,x(i,:),'',u(i,:));
    x(i+1,:) = X(end,:);

    start = false;
    
end
tout = t(1:end_ind);    
xout = x(1:end_ind,:);    
uout = u(1:end_ind,:);    

%% plotting

% transforms

vi = zeros(end_ind,3);
veh_x = zeros(end_ind,3);
veh_y = zeros(end_ind,3);
veh_z = zeros(end_ind,3);

for i = 1:end_ind
    Rib = angle2dcm(xout(i,9), xout(i,8), xout(i,7));
    Rbi = Rib';
    
    Rib_image = angle2dcm(0, xout(i,8), xout(i,7));
    Rbi_image = Rib_image';
    vi(i,:) = (Rbi_image*xout(i,4:6)')';
    
    veh_x(i,:) = (Rbi*[1 0 0]')';
    veh_y(i,:) = (Rbi*[0 1 0]')';
    veh_z(i,:) = (Rbi*[0 0 1]')';
end

N = round(end_ind/10);

veh_x_p = downsample(veh_x,N);
veh_y_p = downsample(veh_y,N);
veh_z_p = downsample(veh_z,N);
xout_ds = downsample(xout,N);
figure
plot3(xout(:,1),xout(:,2),xout(:,3))
hold on
plot3(t_pos(1),t_pos(2),t_pos(3),'r.')
quiver3(xout_ds(:,1),xout_ds(:,2),xout_ds(:,3),veh_x_p(:,1),veh_x_p(:,2),veh_x_p(:,3))
quiver3(xout_ds(:,1),xout_ds(:,2),xout_ds(:,3),veh_y_p(:,1),veh_y_p(:,2),veh_y_p(:,3))
quiver3(xout_ds(:,1),xout_ds(:,2),xout_ds(:,3),veh_z_p(:,1),veh_z_p(:,2),veh_z_p(:,3))
hold off
title('3D Trajectory Plot')
legend('Trajectory','Target Position')
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
axis equal
grid on


% distance to target
target_vec = ones(end_ind,1)*t_pos - xout(:,1:3);

target_dist = sqrt(sum(target_vec.^2,2));

figure
plot(tout,target_dist)
title('Distance to Target')
xlabel('t(s)')
ylabel('Distance to Target')
axis tight
grid on

% speed

speed = sqrt(sum(xout(:,4:6).^2,2));

figure
plot(tout,speed)
title('Vehicle Speed')
xlabel('t(s)')
ylabel('speed (m/s)')
axis tight
grid on

% side slip rate


figure
plot(tout,xout(:,5))
title('Vehicle Side Slip Rate')
xlabel('t(s)')
ylabel('v_y (m/s)')
axis tight
grid on

% forward vehicle speed
speedf= vi(:,1);

figure
plot(tout,speedf)
title('Vehicle Forward Speed')
xlabel('t(s)')
ylabel('v_y (m/s)')
axis tight
grid on