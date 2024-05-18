% Position (m)
px = squeeze(out.yout{1}.Values.Data(1,:,:));
py = squeeze(out.yout{1}.Values.Data(2,:,:));
pz = squeeze(out.yout{1}.Values.Data(3,:,:));
t=out.yout{1}.Values.Time;
figure
plot(t, pz)
hold on
plot(t, py)
plot(t, px)
hold off
legend('dz', 'dy', 'dx')
xlabel('Time(s)')
ylabel('Displacement(m)')
title('Displacement Components')


% 
% 
% Velocity (m/s)
vx=squeeze(out.yout{2}.Values.Data(1,:,:));
vy=squeeze(out.yout{2}.Values.Data(2,:,:));
vz=squeeze(out.yout{2}.Values.Data(3,:,:));
t=out.yout{2}.Values.Time;  

figure
plot(t, vz)
hold on
plot(t, vy)
plot(t, vx)
hold off
legend('vz', 'vy', 'vx')
xlabel('Time(s)')
ylabel('Velocity(m/s)')
title('Velocity Components')

% 
% 
% 
% % Angle (rad)
ax = squeeze(out.yout{3}.Values.Data(1,:,:));
ay = squeeze(out.yout{3}.Values.Data(2,:,:));
az = squeeze(out.yout{3}.Values.Data(3,:,:));
t = out.yout{3}.Values.Time;  

figure
plot(t, az)
hold on
plot(t, ay)
plot(t, ax)
hold off
legend('az', 'ay', 'ax')
xlabel('Time(s)')
ylabel('Angle (rad)')
title('Angular Components')

% 
% 
% 
% 
% 
% % Angular_Velocity (rad/s)
avx = squeeze(out.yout{4}.Values.Data(1,:,:));
avy = squeeze(out.yout{4}.Values.Data(2,:,:));
avz = squeeze(out.yout{4}.Values.Data(3,:,:));
t = out.yout{4}.Values.Time;  

figure
plot(t, avz)
hold on
plot(t, avy)
plot(t, avx)
hold off
legend('avz', 'avy', 'avx')
xlabel('Time(s)')
ylabel('Angular Rate(rad/s)')
title('Angular Rate Components')









