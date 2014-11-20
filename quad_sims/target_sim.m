function target = target_sim(t_pos,x)

if size(t_pos,1) == 1
    t_pos = t_pos';
end


global FoVv FoVh FoVpv FoVph

% % field of view from center in degrees
% FoVh = 40*pi/180;
% FoVv = 40*pi/180;
% 
%     
% % field of view in pixels from center
% FoVph = 320;
% FoVpv = 320;


% vector to target

vec = t_pos - x(1:3)';

% transforms
Rib = angle2dcm(x(9), x(8), x(7));
Rbi = Rib';

% vector relative to aircraft
vec_b = Rib*vec;

az = atan2(vec_b(2),vec_b(1));
% elevation is positive up here
el = atan2(-vec_b(3),vec_b(1));

if abs(az) > FoVh|| abs(el) > FoVv
    target = nan(1,2);
else

    % range to virutal plane in pixels
    rangeh = FoVph/tan(FoVh);
    rangev = FoVpv/tan(FoVv);
    
    target = [rangeh*tan(az) rangev*tan(el)];
    
end
end
