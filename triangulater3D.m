%
% Basically, Range is found by the simplified 
% solution derived from the law of cosines:
%
%    R1 = (D cross u2) / (u1 cross u2)
%
% Then, R1dot is the time derivative of R1. 
%
% The Blackman book also has derivations for 
% R,Rdot uncertainty as a function of variance
% around theta,thetadot at the sensors.
%
clear all; clc;

dt = 0.001;
t = 0:dt:100.0;
pos_end = 10000.0;
s1 = [100;0;0];
s2 = [100;500;0];
D = s2-s1;

dx = pos_end/length(t);
pos = [0:dx:pos_end-dx;...
       1000*ones(length(t),1)';...
       0*ones(length(t),1)'];
vel = [zeros(length(t),1)';...
       dx/dt*ones(length(t),1)';...
       zeros(length(t),1)'];
   
zeroVec = zeros(3,length(t));
u1 = zeroVec;
u2 = zeroVec;
uDot1 = zeroVec;
uDot2 = zeroVec;
estPos1 = zeroVec;
estPos2 = zeroVec;
estVel1 = zeroVec;
estVel2 = zeroVec;
for i=1:length(t)
    
    r1 = pos(:,i)-s1;
    r2 = pos(:,i)-s2;
    u1(:,i) = r1./norm(r1);
    u2(:,i) = r2./norm(r2);
    
    if (i==1)
        uDot1(:,i) = zeros(3,1);
        uDot2(:,i) = zeros(3,1);
    else
        uDot1(:,i) = (u1(:,i)-u1(:,i-1))/dt;
        uDot2(:,i) = (u2(:,i)-u2(:,i-1))/dt;
    end
    
    g     = norm(cross(u1(:,i),u2(:,i)));
    f1    = norm(cross(D,u2(:,i)));
    f2    = norm(cross(D,u1(:,i)));
    r1est = u1(:,i)*f1/g;
    r2est = u2(:,i)*f2/g;
    estPos1(:,i) = r1est+s1;
    estPos2(:,i) = r2est+s2;
    
    %f1Prime = u1(:,i)*norm(cross(D,uDot2(:,i))) ...
    %          +uDot1(:,i)*norm(cross(D,u2(:,i)));
    %f2Prime = u2(:,i)*norm(cross(D,uDot1(:,i))) ...
    %          +uDot2(:,i)*norm(cross(D,u1(:,i)));
    %gPrime  = norm(cross(uDot1(:,i),u2(:,i))+cross(u1(:,i),uDot2(:,i)));
    %estVel1(:,i) = (f1Prime.*g-f1.*gPrime)./g^2;
    %estVel2(:,i) = (f2Prime.*g-f2.*gPrime)./g^2;
    f1Prime = norm(cross(D,uDot2(:,i)));
    gPrime  = norm(cross(uDot1(:,i),u2(:,i))+cross(u1(:,i),uDot2(:,i)));
    estVel1(:,i) = u1(:,i)*(f1Prime*g-f1*gPrime)/g^2+ ...
                   uDot1(:,i)*f1/g;
end
   
figure('Units', 'Normalized', 'OuterPosition', [0 0 1 1]); hold on;
subplot(3,1,1);hold on;title('Relative Position');grid on;
plot(t, pos(1,:), 'k-');
plot(t, estPos1(1,:), 'r--');
plot(t, estPos2(1,:), 'b--');
legend('Truth','Estimate 1', 'Estimate 2');
ylabel('North (m)');
subplot(3,1,2);hold on;grid on;
plot(t, pos(2,:), 'k-');
plot(t, estPos1(2,:), 'r--');
plot(t, estPos2(2,:), 'b--');
ylabel('East (m)');
subplot(3,1,3);hold on;grid on;
plot(t, pos(3,:), 'k-');
plot(t, estPos1(3,:), 'r--');
plot(t, estPos2(3,:), 'b--');
ylabel('Down (m)');

figure('Units', 'Normalized', 'OuterPosition', [0 0 1 1]); hold on;
subplot(3,1,1);hold on;title('Relative Velocity');grid on;
plot(t, vel(1,:), 'k-');
plot(t, estVel1(1,:), 'r--');
plot(t, estVel2(1,:), 'b--');
legend('Truth','Estimate 1', 'Estimate 2');
ylabel('North (m/s)');
subplot(3,1,2);hold on;grid on;
plot(t, vel(2,:), 'k-');
plot(t, estVel1(2,:), 'r--');
plot(t, estVel2(2,:), 'b--');
ylabel('East (m/s)');
subplot(3,1,3);hold on;grid on;
plot(t, vel(3,:), 'k-');
plot(t, estVel1(3,:), 'r--');
plot(t, estVel2(3,:), 'b--');
ylabel('Down (m/s)');
    