%--------------------------------------------------------------------------
%
% triangulationKalman
%
% Austin Smith
% SIE-554A
% 10/13/2017
%
% Triangulates observed vehicle dynamics based on multiple passive sensor
% estimates in NED frame, where each sensor location is known.
%
% Inputs:  current filtered state (x0=[x,y,z,vx,vy,vz])
%          current filtered error (diag(p0)=[xVar,yVar,zVar,vxVar,vyVar,vzVar])
%          observation vectors (x1=[el,elDot,az,azDot]; ...
%                               xN=[...])
%          observation noise (diag(p1)=[elVar,elDotVar,azVar,azDotVar]; ...
%                             diag(pN)=[...])
%          time vector (time=[tObs_prev,tObs,tEst])
%              where the times are: prev. observation, current, and desired
%              filter estimate time
%
% Outputs: new filtered state (xHat=[x,y,z,vx,vy,vz])
%          new filtered error (diag(pHat)=[xVar,yVar,zVar,vxVar,vyVar,vzVar])
%          estimated state (xBar=[x,y,z,vx,vy,vz])
%          estimated error (diag(pBar)=[xVar,yVar,zVar,vxVar,vyVar,vzVar])
%
%--------------------------------------------------------------------------

function [ xHat,pHat,xBar,pBar ] = triangulationKalman( x0,p0,z,R,time )

% Determine sensor availability (2-3 preferred)
[numSensors,~] = size(z);

% Choose best (not near n*pi) gamma, if possible
% ...

% Determine math and filtering for two best sensors
% ...

% Provide extrapolated estimate
% ...

end

