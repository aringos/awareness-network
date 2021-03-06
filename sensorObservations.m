%--------------------------------------------------------------------------
%
% sensorObservations
%
% Anthony Rodriguez
% SIE-554A
% 10/5/2017
%
% Given a vector of target states and a sensor param struct, produce the 
% output y vector observed by the sensor.
%
% Inputs:  sensor input struct (sensor), target state vector (x_all)
%
% Outputs: Full-rank (az/range/rangerate) observations y_all, 
%          observation mask H_0 (ones where observations exist, zeros
%          elsewhere),
%          all measurement covariance matrices R_all
%--------------------------------------------------------------------------

function [ z, z_truth ] = sensorObservations( sensor, x )
   
   relPos = x(1:2)-sensor.pos;
   relVel = x(3:4);
   range  = norm(relPos);
   rDot   = dot(relVel,relPos./range);
   az     = atan2(relPos(2),relPos(1));
   azDot  = (relPos(1)*relVel(2)-relPos(2)*relVel(1))/range^2;
   
   obs_truth = [az;azDot;range;rDot];
   z_truth   = obs_truth(logical(sensor.H));
   
   n   = length(z_truth);
   z = z_truth+randn(n,1).*sqrt(diag(sensor.R));

end

