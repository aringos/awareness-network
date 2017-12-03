function [ x ] = getStateVector( accelVector, d0, v0, dt )

x = zeros(6, size(accelVector,2));
x(:,1) = [d0;v0;accelVector(:,1)];
x(5:6,:) = accelVector;
for i=2:size(accelVector,2)
   x(:,i) = dynamics(x(:,i-1),dt);
   x(5:6,i) = accelVector(:,i);
end

end

