function [] = plotPosition(logFile)

close all

src = tdfread(logFile);
%Take out the time offset 
src.time = src.time - src.time(1);

plot(src.time, src.positionZ);
title('Position (alt)')
ylim([0.0 4.0]);

figure;
plot(src.time, src.throttle, '.');
title('Throttle RC');
ylim([1400 1600]);

figure;
plot(src.time, src.error);
ylim([min(src.error)-0.005 max(src.error)+0.005]);
title('Error');

src.d_z(1) = 0;
for k = 2:length(src.positionZ)
    src.d_z(k) = (src.positionZ(k) - src.positionZ(k-1))/(src.time(k) - src.time(k-1));
end

figure;
plot(src.time, src.d_z);
title('Position Derivative (m/s)')
end
