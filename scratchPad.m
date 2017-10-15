clear all; clc;

dt = 0.001;
tend = 5.0;
t = 0:dt:tend;

sensor = getSensorModel('Delphi_Mid_ESR', [10;10;0], 10*pi/180, 2*pi/180, 2);
accel  = vehicleMotion( 'carTurnCity', dt, tend );

x = zeros(9, size(accel,2));
x(:,1) = [0;0;0;0;0;0;accel(:,1)];
x(7:9,:) = accel;
for i=2:size(accel,2)
   x(:,i) = dynamics(x(:,i-1),dt);
   x(7:9,i) = accel(:,i);
end

figure; 
subplot(3,2,1);plot(t,x(1,:));
subplot(3,2,3);plot(t,x(2,:));
subplot(3,2,5);plot(t,x(3,:));
subplot(3,2,2);plot(t,x(4,:));
subplot(3,2,4);plot(t,x(5,:));
subplot(3,2,6);plot(t,x(6,:));