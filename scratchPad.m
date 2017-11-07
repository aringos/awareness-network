clear all; clc; close all;

mph2mps = unitsratio('m','mi')/3600;

% Set up true dynamics
dt = 0.001;
tend = 5.0;
t = 0:dt:tend;

% Get sensor and dynamics model
sensor = getSensorModel('Delphi_Mid_ESR', [10;10], 10*pi/180, 2);
accel  = vehicleMotion( 'cruise', dt, tend );

% Simulate dynamics
x = zeros(6, size(accel,2));
x(:,1) = [0;0;20*mph2mps;0;accel(:,1)];
x(5:6,:) = accel;
for i=2:size(accel,2)
   x(:,i) = dynamics(x(:,i-1),dt);
   x(5:6,i) = accel(:,i);
end

% Set up estimation
tEst = 0:sensor.dt:tend;
obs = zeros(3,length(tEst));
obsTrue = obs;
xEst = zeros(4,length(t));
pEst = zeros(4,4,length(t));
j = 1; updateIdx = 1;

% Filter initialization
[obs(:,1),obsTrue(:,1)] = sensorObservations(sensor,x(:,1));
[xEst(:,1),pEst(:,:,1)] = ...
       commonKalman(xEst(:,1),pEst(:,:,1),[0;0],obs(:,1),sensor.R,[0,0,0],0,1);

% Simulate estimation
for i=2:length(t)
   
   if mod(i-1,sensor.dt/dt)==0
       j=j+1;
       updateIdx = i;
       [obs(:,j),obsTrue(:,j)] = sensorObservations(sensor,x(:,i));
       update = 1;
       tIn = [tEst(j-1),tEst(j),tEst(j)];
   else
       update = 0;
       tIn = [0,tEst(j),t(i)];
   end
   [xEst(:,i),pEst(:,:,i)] = ...
       commonKalman(xEst(:,updateIdx),pEst(:,:,updateIdx),[0,0],obs(:,j),sensor.R,tIn,update,0);
end

figure('name','True Dynamics'); 
subplot(3,2,1);plot(t,x(1,:));
subplot(3,2,3);plot(t,x(2,:));
subplot(3,2,5);plot(t,x(3,:));
subplot(3,2,2);plot(t,x(4,:));
subplot(3,2,4);plot(t,x(5,:));
subplot(3,2,6);plot(t,x(6,:));

figure('name','Observations');
subplot(3,1,1);plot(tEst,obs(1,:),tEst,obsTrue(1,:));
subplot(3,1,2);plot(tEst,obs(2,:),tEst,obsTrue(2,:));
subplot(3,1,3);plot(tEst,obs(3,:),tEst,obsTrue(3,:));

figure('name','Relative States');
subplot(4,1,1);plot(t,xEst(1,:),t,x(1,:)-ones(1,length(t)).*sensor.pos(1));
subplot(4,1,2);plot(t,xEst(2,:),t,x(2,:)-ones(1,length(t)).*sensor.pos(2));
subplot(4,1,3);plot(t,xEst(3,:),t,x(3,:));
subplot(4,1,4);plot(t,xEst(4,:),t,x(4,:));

