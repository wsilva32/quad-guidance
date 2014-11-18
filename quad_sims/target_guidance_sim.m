%% Simulation
clear all
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

% max thrust (in real system will be approximate as linear from thrust for
% level flight)
T_max = 50;

CDA = 1*0.15;

% target position
t_pos = [100 20 -10];

% desired vehicle speed
speed_des = 2;

% time settings
tmax = 300;
rate = 100;

% hover thrust
th0 = m*g/T_max;

% GAINS

global KP_t KI_t KD_t KP_p KI_p KD_p KP_r KI_r KD_r KP_rl KI_rl KD_rl

% throttle
KP_t = 1;
KI_t = 0;
KD_t = 0;

% pitch
KP_p = 0.05;
KI_p = 0;
KD_p = 0;

% body z rotation rate
KP_r = 1;
KI_r = 0;
KD_r = 0;

% roll
KP_rl = 1;
KI_rl = 0;
KD_rl = 0;

% SIMUALTION SETUP

% initial state (hover)
x0 = [0 0 0 0 0 0 0 0 0 0 th0];

% time sequence
dt = 1/rate;
t = 1:dt:tmax;

% loops
n = length(t);

% preallocate state
u = zeros(n,4);
x = zeros(n,length(x0));
x(1,:) = x0;

start = true;
end_ind = n;
for i = 1:n-1
    
    target = target_sim(t_pos,x(i,:));
    try
        u(i,:) = quad_control(t(i),target,x(i,:),speed_des,start);
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
figure
plot3(xout(:,1),xout(:,2),xout(:,3))
hold on
plot3(t_pos(1),t_pos(2),t_pos(3),'r.')
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