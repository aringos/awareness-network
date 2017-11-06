clear all; clc; close all;

mph2mps = unitsratio('m','mi')/3600;

dt = 0.001;
tend = 5.0;
t = 0:dt:tend;

sensor = getSensorModel('Delphi_Mid_ESR', [10;10;0], 10*pi/180, 2*pi/180, 2);
accel  = vehicleMotion( 'cruise', dt, tend );

x = zeros(9, size(accel,2));
x(:,1) = [0;0;0;20*mph2mps;0;0;accel(:,1)];
x(7:9,:) = accel;
for i=2:size(accel,2)
   x(:,i) = dynamics(x(:,i-1),dt);
   x(7:9,i) = accel(:,i);
end

figure('name','Truth'); 
subplot(3,2,1);plot(t,x(1,:));
subplot(3,2,3);plot(t,x(2,:));
subplot(3,2,5);plot(t,x(3,:));
subplot(3,2,2);plot(t,x(4,:));
subplot(3,2,4);plot(t,x(5,:));
subplot(3,2,6);plot(t,x(6,:));

obs = zeros(3,length(t));
xEst = zeros(4,length(t));
pEst = zeros(4,4,length(t));
xEst(:,1) = x([1,2,4,5],1)-[10;10;0;0];
pEst(:,:,1) = zeros(4);
pEst(1,1,1) = 5;  pEst(2,2,1) = 5;
pEst(3,3,1) = .5;  pEst(4,4,1) = .5;
for i=2:length(t)
   relPos = x(1:3,i)-sensor.pos;
   range  = norm(relPos);
   rDot   = dot(x(4:6,i),relPos./range);
   az     = atan2(relPos(2),relPos(1));
   obs(:,i) = [az;range;rDot]+randn(3,1).*diag(sensor.R);
   [xEst(:,i),pEst(:,:,i)] = ...
       commonKalman(xEst(:,i-1),pEst(:,:,i-1),0,[0,0],obs(:,i),sensor.R,[t(i-1),t,t]);
end

figure('name','Observations');
subplot(3,1,1);plot(t,obs(1,:));
subplot(3,1,2);plot(t,obs(2,:));
subplot(3,1,3);plot(t,obs(3,:));

figure('name','Estimated Relative');
subplot(4,1,1);plot(t,xEst(1,:));
subplot(4,1,2);plot(t,xEst(2,:));
subplot(4,1,3);plot(t,xEst(3,:));
subplot(4,1,4);plot(t,xEst(4,:));

