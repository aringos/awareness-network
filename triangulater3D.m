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
s2 = [200;200;0];
D = s2-s1;
D_mag = norm(D);


dx = pos_end/length(t);
pos = [0:dx:pos_end-dx;...
       1000*ones(length(t),1)';...
       500*ones(length(t),1)'];
vel = [dx/dt*ones(length(t),1)';...
       zeros(length(t),1)';...
       zeros(length(t),1)'];
   
f = figure; hold on;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);

u1 = zeros(length(t),3);
u2 = u1;
uDot1 = u1;
uDot2 = u1;
estPos = u1;
estVel = u1;
for i=1:length(t)
    
    r1 = pos(:,i)-s1(:,i);
    r2 = pos(:,i)-s2(:,i);
    u1(:,i) = r1./norm(r1);
    u2(:,i) = r2./norm(r2);
    
    if (i==1)
        uDot1(:,i) = zeros(3,1);
        uDot2(:,i) = zeros(3,1);
    else
        uDot1(:,i) = (u1(:,i)-u1(:,i-1))/dt;
        uDot2(:,i) = (u2(:,i)-u2(:,i-1))/dt;
    end
    
    r1est = u1(:,i)*norm(cross(D,u2(:,i)))/norm(cross(u2(:,i),u1(:,i)));
    r2est = u2(:,i)*norm(cross(D,u1(:,i)))/norm(cross(u1(:,i),u2(:,i)));
    estPos(:,i) = u1(:,i)+s1;
end
    
subplot(4,1,1);hold on;title('Range');grid on;
plot(t_elapsed, r1_mag, 'r-');
plot(t_elapsed, r2_mag, 'b-');
plot(t_elapsed, r1est_mag, 'r--', 'LineWidth', 2);
plot(t_elapsed, r2est_mag, 'b--', 'LineWidth', 2);
subplot(4,1,2);hold on; title('RangeRate');grid on;
plot(t_elapsed, r1dot_mag, 'r-');
plot(t_elapsed, r2dot_mag, 'b-');
plot(t_elapsed, r1dot_est, 'r--', 'LineWidth', 2);
plot(t_elapsed, r2dot_est, 'b--', 'LineWidth', 2);
subplot(2,1,2);hold on; title('Geometry');grid on;
scatter(pos(1,i), pos(2,i), 'k.');
scatter(s1(1,1),s1(2,1), 'rO');
scatter(s2(1,1),s2(2,1), 'bO');
plot(r1(1,:), r1(2,:), 'r-');
plot(r2(1,:), r2(2,:), 'b-');
    