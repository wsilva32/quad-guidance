function xdot = simple_quad_dyn(t,x,u)
% u = [pitch_com roll_com r_com throttle_com]'
% where r is the rate about body z (need to verify this is correct)
% x = [x y z u v w roll pitch yaw r throttle]
global FoVv FoVh FoVpv FoVph T_max m g CDA
% input parsing
roll_com = u(2);
pitch_com = u(1);
r_com = u(3);
throttle_com = u(4);

% time delay for command inner loops
% time constants
t_const_roll = 1;
t_const_pitch = 1;
t_const_yaw_rate = 0.3;
t_const_throttle = 0.05;

roll_dot =  (1./t_const_roll).*(roll_com - x(7));
pitch_dot =  (1./t_const_pitch).*(pitch_com - x(8));
r_dot =  (1./t_const_yaw_rate).*(r_com - x(10));
throttle_dot =  (1./t_const_throttle).*(throttle_com - x(11));

% % constants
% g = 9.807;
% m = 2;
% CDA = 1*0.15;
% max thrust (in real system will be approximate as linear from thrust for
% level flight)
% T_max = 50;

% force balance

% in body z

Rib = angle2dcm(x(9), x(8), x(7));
Rbi = Rib';
% gravity force
gb = Rib*[0 0 g*m]';

% thrust force
T = [0 0 -x(11)*T_max]';

% drag force

if norm([x(4) x(5) x(6)]')>0
    D = -CDA*norm([x(4) x(5) x(6)]')^2/2*([x(4) x(5) x(6)]'/norm([x(4) x(5) x(6)]'));
else
    D = zeros(3,1);
end

Fb = T + D + gb;

% velocity

Vi = Rbi*[x(4) x(5) x(6)]';

% euler rate from rotation about body z

E_mat = [1 sin(x(7))*sin(x(8))  cos(x(7))*tan(x(8));
        0 cos(x(7)) -sin(x(7));
        0 sin(x(7))/cos(x(8)) cos(x(7))/cos(x(8))];

euler_dot = [roll_dot pitch_dot 0]' + E_mat*[0 0 x(10)]';

xdot = [Vi; Fb/m; euler_dot; r_dot; throttle_dot];

end
