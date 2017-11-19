function [ x ] = getStateVector( accelVector, dt )

mph2mps = unitsratio('m','mi')/3600;
x = zeros(6, size(accelVector,2));
x(:,1) = [0;0;0;20*mph2mps;accelVector(:,1)];
x(5:6,:) = accelVector;
for i=2:size(accelVector,2)
   x(:,i) = dynamics(x(:,i-1),dt);
   x(5:6,i) = accelVector(:,i);
end

end

