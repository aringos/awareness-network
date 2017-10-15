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
% Outputs: Full-rank (az/el/range/rangerate) observations y_all, 
%          observation mask H_0 (ones where observations exist, zeros
%          elsewhere),
%          all measurement covariance matrices R_all
%--------------------------------------------------------------------------

function [ y, y_truth ] = sensorObservations( sensor, x, t )
   
    

end

