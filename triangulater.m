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
s1 = [100;0];
s2 = [200;200];
D = s2-s1;
D_mag = norm(D);


dx = pos_end/length(t);
pos = [0:dx:pos_end-dx';...
       1000*ones(length(t),1)'];
   
f = figure; hold on;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
r1_mag = [];
r1est_mag = [];
r2_mag = [];
r2est_mag = [];
r1dot_mag = [];
r2dot_mag = [];
r1dot_est = [];
r2dot_est = [];
t_elapsed = [];
theta1 = atan2(pos(2,1)-s1(2,1), pos(1,1)-s1(1,1));
theta2 = atan2(pos(2,1)-s2(2,1), pos(1,1)-s2(1,1));
for i=1:length(t)
    theta1prev = theta1;
    theta1 = atan2(pos(2,i)-s1(2,1), pos(1,i)-s1(1,1));
    theta1dot = (theta1-theta1prev)/dt;
    u1 = [cos(theta1);sin(theta1)];
    u1dot = [-theta1dot*u1(2); theta1dot*u1(1)];
    theta2prev = theta2;
    theta2 = atan2(pos(2,i)-s2(2,1), (pos(1,i)-s2(1,1)));
    theta2dot = (theta2-theta2prev)/dt;
    u2 = [cos(theta2);sin(theta2)];
    u2dot = [-theta2dot*u2(2); theta2dot*u2(1)];
    t_elapsed = [t_elapsed t(i)];
    r1 = [linspace(s1(1,1),pos(1,i),100);...
          linspace(s1(2,1),pos(2,i),100)];
    r1est_mag = [r1est_mag (D(1)*u2(2)-D(2)*u2(1))/(u1(1)*u2(2)-u1(2)*u2(1))];
    r1_mag = [r1_mag norm([pos(1,i)-s1(1,1); pos(2,i)-s1(2,1)])];
    r2 = [linspace(s2(1,1),pos(1,i),100);...
          linspace(s2(2,1),pos(2,i),100)];
    r2_mag = [r2_mag norm([pos(1,i)-s2(1,1); pos(2,i)-s2(2,1)])];
    r2est_mag = [r2est_mag (D(1)*u1(2)-D(2)*u1(1))/(u1(1)*u2(2)-u1(2)*u2(1))];
    f1 = D(1)*u2(2)-D(2)*u2(1);
    g1 = u1(1)*u2(2)-u1(2)*u2(1);
    f1prime = D(1)*u2dot(2)-D(2)*u2dot(1);
    g1prime = u1dot(1)*u2(2)+u1(1)*u2dot(2)-u1dot(2)*u2(1)-u1(2)*u2dot(1);
    r1dot_est = [r1dot_est (f1prime*g1-f1*g1prime) / (g1*g1)];
    f2 = D(1)*u1(2)-D(2)*u1(1);
    g2 = u1(1)*u2(2)-u1(2)*u2(1);
    f2prime = D(1)*u1dot(2)-D(2)*u1dot(1);
    g2prime = u1dot(1)*u2(2)+u1(1)*u2dot(2)-u1dot(2)*u2(1)-u1(2)*u2dot(1);
    r2dot_est = [r2dot_est (f2prime*g2-f2*g2prime) / (g2*g2)];
    if (i==1)
        r1dot_mag = [0];
        r2dot_mag = [0];
    else
        r1dot_mag = [r1dot_mag (r1_mag(1,i)-r1_mag(1,i-1))/dt];
        r2dot_mag = [r2dot_mag (r2_mag(1,i)-r2_mag(1,i-1))/dt];
    end
    clf(f);   
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
    pause(0.0001);
end