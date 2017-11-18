clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameterizations go here. In the future, this should be a function 
% where these are passed in (so that we can loop it)

dt = 0.001;
tend = 5.0;
t = 0:dt:tend;

sensors = [getSensorModel('R20A', [200;60], 200*pi/180, 2)];
sensors = [sensors getSensorModel('R20A', [300;-60], 160*pi/180, 2)];
sensors = [sensors getSensorModel('R20A', [400;60], 200*pi/180, 2)];
sensors = [sensors getSensorModel('R20A', [500;-60], 160*pi/180, 2)];
sensors = [sensors getSensorModel('R20A', [600;60], 200*pi/180, 2)];
accel  = vehicleMotion( 'cruise', dt, tend );          


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FOVvertices = drawSensorFOVs(sensors);

%Form the dynamics truth
x = zeros(9, size(accel,2));
x(:,1) = [0;0;0;0;0;0;accel(:,1)];
x(7:9,:) = accel;
for i=2:size(accel,2)
   x(:,i) = dynamics(x(:,i-1),dt);
   x(7:9,i) = accel(:,i);
end

%Form observation sets for all sensors
observations = [];
for s=1:length(sensors)
   observations = [observations getObservations(sensors(s),x,t,dt)];
end

%Fusion algorithm should go here
%doSomeKindOfFusion();

figure; 
subplot(3,2,1); hold on; title('Position (x)'); 
plot(t,x(1,:), 'k--', 'LineWidth', 2);
subplot(3,2,3); hold on; title('Position (y)'); 
plot(t,x(2,:), 'k--', 'LineWidth', 2);
subplot(3,2,5); hold on; title('Position (z)'); 
plot(t,x(3,:), 'k--', 'LineWidth', 2);
subplot(3,2,2); hold on; title('Velocity (x)'); 
plot(t,x(4,:), 'k--', 'LineWidth', 2);
subplot(3,2,4); hold on; title('Velocity (y)'); 
plot(t,x(5,:), 'k--', 'LineWidth', 2);
subplot(3,2,6); hold on; title('Velocity (z)'); 
plot(t,x(6,:), 'k--', 'LineWidth', 2);