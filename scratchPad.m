clear all; clc; close all;

mph2mps = unitsratio('m','mi')/3600;

% Set up true dynamics
dt = 1/1500;
tend = 5.0;
t = 0:dt:tend;

% Simulate dynamics
accel  = vehicleMotion( 'cruise', dt, tend );
x = zeros(6, size(accel,2));
x(:,1) = [0;0;20*mph2mps;0;accel(:,1)];
x(5:6,:) = accel;
for i=2:size(accel,2)
   x(:,i) = dynamics(x(:,i-1),dt);
   x(5:6,i) = accel(:,i);
end

% % Set up radar estimation
% pos = [10;10];
% sensor = getSensorModel('Delphi_Mid_ESR', pos, 10*pi/180, 2);
% tEst = 0:sensor.dt:tend;
% obs = zeros(3,length(tEst));
% obsTrue = obs;
% xEst = zeros(4,length(t));
% pEst = zeros(4,4,length(t));
% j = 1; updateIdx = 1;
% 
% % Radar filter initialization
% [obs(:,1),obsTrue(:,1)] = sensorObservations(sensor,x(:,1));
% [xEst(:,1),pEst(:,:,1)] = ...
%        commonKalman(xEst(:,1),pEst(:,:,1),[0;0],obs(:,1),sensor.R,[0,0,0],0,1);
% 
% % Simulate radar estimation
% for i=2:length(t)
%    
%    if mod(i-1,sensor.dt/dt)==0
%        j=j+1;
%        updateIdx = i;
%        [obs(:,j),obsTrue(:,j)] = sensorObservations(sensor,x(:,i));
%        update = 1;
%        tIn = [tEst(j-1),tEst(j),tEst(j)];
%    else
%        update = 0;
%        tIn = [0,tEst(j),t(i)];
%    end
%    [xEst(:,i),pEst(:,:,i)] = ...
%        commonKalman(xEst(:,updateIdx),pEst(:,:,updateIdx),[0,0],obs(:,j),sensor.R,tIn,update,0);
% end
% 
% figure('name','True Dynamics'); 
% subplot(3,2,1);plot(t,x(1,:));
% subplot(3,2,3);plot(t,x(2,:));
% subplot(3,2,5);plot(t,x(3,:));
% subplot(3,2,2);plot(t,x(4,:));
% subplot(3,2,4);plot(t,x(5,:));
% subplot(3,2,6);plot(t,x(6,:));
% 
% figure('name','Radar Observations');
% subplot(3,1,1);plot(tEst,obs(1,:),tEst,obsTrue(1,:));
% subplot(3,1,2);plot(tEst,obs(2,:),tEst,obsTrue(2,:));
% subplot(3,1,3);plot(tEst,obs(3,:),tEst,obsTrue(3,:));
% 
% figure('name','Radar Relative States');
% subplot(4,1,1);plot(t,xEst(1,:),t,x(1,:)-ones(1,length(t)).*sensor.pos(1));
% subplot(4,1,2);plot(t,xEst(2,:),t,x(2,:)-ones(1,length(t)).*sensor.pos(2));
% subplot(4,1,3);plot(t,xEst(3,:),t,x(3,:));
% subplot(4,1,4);plot(t,xEst(4,:),t,x(4,:));

% Set up triangulation estimation
pos1 = [10;10]; pos2 = [10;30];
sensor1 = getSensorModel('R20A', pos1, 10*pi/180, 2);
sensor2 = getSensorModel('R20A', pos2, 10*pi/180, 2);
tEst = 0:sensor1.dt:tend;
d = pos2-pos1;
obs = zeros(4,length(tEst));
obsTrue = obs;
xEst = zeros(8,length(t));
pEst = zeros(8,8,length(t));
j = 1; updateIdx = 1;

% Triangulation filter initialization
[obs(1:2,1),obsTrue(1:2,1)] = sensorObservations(sensor1,x(:,1));
[obs(3:4,1),obsTrue(3:4,1)] = sensorObservations(sensor2,x(:,1));
R1and2 = zeros(4); R1and2(1:2,1:2) = sensor1.R; R1and2(3:4,3:4) = sensor2.R;
[xEst(:,1),pEst(:,:,1)] = ...
       commonKalman(xEst(:,1),pEst(:,:,1),d,obs(:,1),R1and2,[0,0,0],0,1);

% Simulate triangulation estimation
for i=2:length(t)
   
   if mod(i-1,sensor1.dt/dt)==0
       j=j+1;
       updateIdx = i;
       [obs(1:2,j),obsTrue(1:2,j)] = sensorObservations(sensor1,x(:,i));
       [obs(3:4,j),obsTrue(3:4,j)] = sensorObservations(sensor2,x(:,i));
       update = 1;
       tIn = [tEst(j-1),tEst(j),tEst(j)];
   else
       update = 0;
       tIn = [0,tEst(j),t(i)];
   end
   [xEst(:,i),pEst(:,:,i)] = ...
       commonKalman(xEst(:,updateIdx),pEst(:,:,updateIdx),d,obs(:,j),R1and2,tIn,update,0);
end

figure('name','Passive Observations');
subplot(4,1,1);plot(tEst,obs(1,:),tEst,obsTrue(1,:));
subplot(4,1,2);plot(tEst,obs(2,:),tEst,obsTrue(2,:));
subplot(4,1,3);plot(tEst,obs(3,:),tEst,obsTrue(3,:));
subplot(4,1,4);plot(tEst,obs(4,:),tEst,obsTrue(4,:));

figure('name','Triangulate Relative States');
subplot(4,2,1);plot(t,xEst(1,:),t,x(1,:)-ones(1,length(t)).*sensor1.pos(1));
subplot(4,2,2);plot(t,xEst(2,:),t,x(2,:)-ones(1,length(t)).*sensor1.pos(2));
subplot(4,2,3);plot(t,xEst(3,:),t,x(3,:));
subplot(4,2,4);plot(t,xEst(4,:),t,x(4,:));
subplot(4,2,5);plot(t,xEst(5,:),t,x(1,:)-ones(1,length(t)).*sensor2.pos(1));
subplot(4,2,6);plot(t,xEst(6,:),t,x(2,:)-ones(1,length(t)).*sensor2.pos(2));
subplot(4,2,7);plot(t,xEst(7,:),t,x(3,:));
subplot(4,2,8);plot(t,xEst(8,:),t,x(4,:));


