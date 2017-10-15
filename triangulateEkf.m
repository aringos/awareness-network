%--------------------------------------------------------------------------
%
% triangulateEkf
%
% Austin Smith
% SIE-554A
% 10/13/2017
%
% Triangulates observed vehicle dynamics based on multiple passive sensor
% estimates in NED frame, where each sensor location is known. This version
% is an extended Kalman filter.
%
% Inputs:  current filtered state (x0=[x,y,z,vx,vy,vz])
%          current filtered error (diag(p0)=[xVar,yVar,zVar,vxVar,vyVar,vzVar])
%          observation vectors (z1=[el,elDot,az,azDot]; ...
%                               z2=[...])
%          observation noise (diag(r1)=[elVar,elDotVar,azVar,azDotVar]; ...
%                             diag(r2)=[...])
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

function [ xHat,pHat,xBar,pBar ] = triangulateEkf( x0,p0,z,r,time )

% Choose best (not near n*pi) gamma, if possible
% Choose best (not +/-pi/2) elevations, if possible
% Do this outside the filter!

% Extended Kalman filter
%
% Predicted at measurement:
% xBar=Phi*x0
% pBar=Phi*p0*Phi'+Q
% zBar=H*xBar
% rBar=H*pBar*H'
%
% Filtering:
% y=z-zBar
% S=r+rBar
% K=pBar*H'*S^-1
% xHat=xBar+K*y
% pHat=(I-K*H)*pBar
%
% Extrapolated estimate:
% ...Same eqns as predicted

% Predicted measurements and states
% ...

% Measurement matrix
% ...

% Filter measurements
% ...

% Extrapolate estimate
% ...

end

