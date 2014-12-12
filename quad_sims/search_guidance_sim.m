%% Simulation
% clear all

% INPUT AND STATE DESCRIPTION
% u = [pitch_com roll_com r_com throttle_com]'
% where r is the rate about body z (need to verify this is correct)
% x = [x y z u v w roll pitch yaw r throttle]

% TIME SEQUENCE

% time settings
tmax = 100;
rate = 10;
target_rate = 10;

dt = 1/rate;
t = 0:dt:tmax;

% desired vehicle speed
speed_des = 1;

% PARAMETERS

global T_max m g CDA


% gravity
g = 9.807;

% mass
m = 1;

% measurement noise

% turn on measurment noise
noise = true;

sig_pos = 0.01;
sig_vel = 0.01;
sig_euler = 0.1*pi/180;
sig_r = 0.2*pi/180;
sig_X = [sig_pos*ones(1,3) sig_vel*ones(1,3) sig_euler*ones(1,3) sig_r 0]; %no noise added to the throttle state because this is not measured 

sig_target = 2;

% time delays (first order model)
global t_const_roll t_const_pitch t_const_yaw_rate t_const_throttle
% time constants
t_const_roll = 0.1;
t_const_pitch = 0.1;
t_const_yaw_rate = 0.01;
t_const_throttle = 0.01;


% wind disturbances

global sig_w_x sig_w_y sig_w_z Wd
sig_w_x = 0.0*dt;
sig_w_y = 0.0*dt;
sig_w_z = 0.0*dt;
Wd = 0;



% maybe make into three element vector so vertical component is greater
CDA = 1*0.15;

% hover thrust fraction
global th0
th0 = 0.6;

% max thrust (in real system will be approximate as linear from thrust for
% level flight)
T_max = m*g/th0;

% GAINS

global KP_alt KI_alt KD_alt KP_roll KI_roll KD_roll KP_pitch KI_pitch KD_pitch

% throttle
KP_alt =  0.065;
KI_alt = 0;
KD_alt = 0.1;

% roll
KP_roll = 2*pi/180;
KI_roll = 0.0;
KD_roll = 7*pi/180;

% pitch
KP_pitch = 2*pi/180;
KI_pitch = 0;
KD_pitch = 7*pi/180;

% SIMULATION SETUP

% initial state (hover)
x0 = [0 0 0 0 0 0 0 0 0 20*pi/180 th0];

% loops
n = length(t);
l = length(x0);
% preallocate state
u = zeros(n,4);
x = zeros(n,length(x0));
x(1,:) = x0;

start = true;
end_ind = n;
M = 0;
target = [1,2,3];
for i = 1:n-1
    i
    if noise
        x_meas = x(i,:) + (diag(sig_X.^2)*randn(l,1))';
    else
        x_meas = x(i,:);
    end
    if i == 2092
        1
    end
    
    u(i,:) = search_control(t(i),target,x_meas,start);
%     else
%         u(i,:) = u(i-1,:);
%     end
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

N = round(end_ind/20);

veh_x_p = downsample(veh_x,N);
veh_y_p = downsample(veh_y,N);
veh_z_p = downsample(veh_z,N);
xout_ds = downsample(xout,N);
figure
plot3(xout(:,1),xout(:,2),xout(:,3))
hold on
plot3(target(1),target(2),target(3),'r.')
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
target_vec = ones(end_ind,1)*target - xout(:,1:3);

target_dist = sqrt(sum(target_vec.^2,2));

figure
plot(tout,target_vec(:,3))
title('Alt Err')
xlabel('t(s)')
ylabel('Distance to Target')
axis tight
grid on

% % speed
% 
% speed = sqrt(sum(xout(:,4:6).^2,2));
% 
figure
plot(tout,target_vec(:,2))
title('Y Err')
xlabel('t(s)')
ylabel('speed (m/s)')
axis tight
grid on

% % side slip rate
% 
% 
figure
plot(tout,target_vec(:,1))
title('X Err')
xlabel('t(s)')
ylabel('v_y (m/s)')
axis tight
grid on
% 
% % forward vehicle speed
% speedf= vi(:,1);
% 
% figure
% plot(tout,speedf)
% title('Vehicle Forward Speed')
% xlabel('t(s)')
% ylabel('v_y (m/s)')
% axis tight
% grid on