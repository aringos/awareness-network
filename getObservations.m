function [ observations ] = getObservations( sensor, x, t, dt )

%State vector: [az azRate range rangeRate]

observations = [];
modulo = sensor.dt / dt;
counter = 1;
x_ = [];
y_ = [];
t_ = [];
for k=1:length(t)
    if (mod(counter,modulo)==0)
       obs.t = t(k);
       obs.y = [0.0 0.0 0.0 0.0];
       obs.x = [0.0 0.0 0.0 0.0];
       relState = [x(1,k)-sensor.pos(1,1); ...
                   x(2,k)-sensor.pos(2,1); ...
                   x(4,k); ...
                   x(5,k)];
       obs.x(1) = tan(relState(3)/norm(relState(1:2)));
       obs.x(2) = tan(relState(2)/relState(1));
       obs.x(3) = norm(relState(1:3));
       obs.x(4) = dot(relState(1:3), relState(4:6));
       x_ = [x_; obs.x];
       obs.y(1) = normrnd(obs.x(1), sensor.R(1,1));
       obs.y(2) = normrnd(obs.x(2), sensor.R(2,2));
       obs.y(3) = normrnd(obs.x(3), sensor.R(3,3));
       obs.y(4) = normrnd(obs.x(4), sensor.R(4,4));
       y_ = [y_; obs.y];
       t_ = [t_; obs.t];
       observations = [observations obs];
    end
    counter = counter + 1;
end

figure;
plots = [];
plots = [plots subplot(4,1,1)]; 
hold on; title('Elevation (deg)');
plot(t_, x_(:,1).*180/pi, 'k--');
plot(t_, y_(:,1).*180/pi, 'b-');
plots = [plots subplot(4,1,2)]; 
hold on; title('Azimuth (deg)');
plot(t_, x_(:,2).*180/pi, 'k--');
plot(t_, y_(:,2).*180/pi, 'b-');
plots = [plots subplot(4,1,3)]; 
hold on; title('Range (m)');
plot(t_, x_(:,3), 'k--');
plot(t_, y_(:,3), 'b-');
plots = [plots subplot(4,1,4)]; 
hold on; title('RangeRate (m/s)');
plot(t_, x_(:,4), 'k--');
plot(t_, y_(:,4), 'b-');
linkaxes(plots, 'x');

figure;
plots = [];
plots = [plots subplot(4,1,1)]; 
hold on; title('Elevation Error(deg)');
plot(t_, y_(:,1).*180/pi - x_(:,1).*180/pi, 'r-');
plots = [plots subplot(4,1,2)]; 
hold on; title('Azimuth Error(deg)');
plot(t_, y_(:,2).*180/pi - x_(:,2).*180/pi, 'r-');
plots = [plots subplot(4,1,3)]; 
hold on; title('Range Error(m)');
plot(t_, y_(:,3).*180/pi - x_(:,3).*180/pi, 'r-');
plots = [plots subplot(4,1,4)]; 
hold on; title('RangeRate Error(m/s)');
plot(t_, y_(:,4).*180/pi - x_(:,4).*180/pi, 'r-');
linkaxes(plots, 'x');
end

