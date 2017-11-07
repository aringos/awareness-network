%--------------------------------------------------------------------------
%
% commonKalman
%
% Austin Smith
% SIE-554A
% 11/03/2017
%
% Filters 2-D radar observations (az,range,range rate) and estimates
% vehicle dynamics.
%
% OR
% 
% Triangulates 2-D vehicle dynamics estimate based on multiple passive sensor
% observations (az1,az1Dot,az2,az2Dot), where each sensor location is
% known.
%
% In either case, vehicle state estimates are in local NED frame relative
% to the sensor(s)
%
% Inputs:  current filtered state (x0=[x,y,vx,vy] or [x1,y1,vx1,vy1,...])
%          current filtered error (diag(p0)=[xVar,yVar,vxVar,vyVar] or
%                                           [x1Var,y1Var,vx1Var,vy1Var,...])
%          sensor2 position rel. sensor1 (d=[x21,y21])
%          observation vector (z=[meas1,meas2,meas3,...])
%          observation noise (diag(r)=[var1,var2,var3,...])
%          time vector (time=[tObsPrev,tObs,tEst])
%              where the times are: prev. observation, current, and desired
%              filter estimate time
%
% Outputs: new filtered state (xHat=[x,y,vx,vy] or [x1,y1,vx1,vy1,...])
%          new filtered error (diag(pHat)=[xVar,yVar,vxVar,vyVar] or 
%                                         [x1Var,y1Var,vx1Var,vy1Var,...])
%          estimated state (xBar=[x,y,vx,vy] or [x1,y1,vx1,vy1,...])
%          estimated error (diag(pBar)=[xVar,yVar,vxVar,vyVar] or
%                                      [x1Var,y1Var,vx1Var,vy1Var,...])
%
%--------------------------------------------------------------------------

function [xFilt,pFilt,xExtrap,pExtrap] = commonKalman(x0,p0,d,z,r,time,update,init)

% Choose best (not near n*pi) gamma, if possible
% Choose best (not +/-pi/2) elevations, if possible
% Do this outside the filter!

dtPred = time(2)-time(1);
dtExtrap = time(3)-time(2);

if length(z)==3
    sensorType = 0;
else
    sensorType = 1;
end

switch sensorType
    
    % Radar sensor
    case 0
        if init==1
            [xFilt,pFilt] = radarInit(z,r);
            return;
        end
        
        if update==1
            phiPred = radarStateTransitionMatrix(dtPred);
            qPred = radarProcessNoise(dtPred);
            [xPred,pPred] = extrapolateEstimate(x0,p0,phiPred,qPred);
            H = radarMeasurementMatrix(xPred);
            [zPred,rPred] = predictRadarMeasurement(xPred,pPred,H);

            [xFilt,pFilt] = filterEstimate(xPred,pPred,z,zPred,r,rPred,H);
        else
            xFilt = x0;  pFilt = p0;
        end
        
        phiExtrap = radarStateTransitionMatrix(dtExtrap);
        qExtrap = radarProcessNoise(dtExtrap);
        [xExtrap,pExtrap] = extrapolateEstimate(xFilt,pFilt,phiExtrap,qExtrap);
    
    % Two passive sensors
    case 1
        if init==1
           [xFilt,pFilt] = triangulateInit(z,r,d);
        end
        
        if update==1
            phiPred = triangulateStateTransitionMatrix(dtPred);
            qPred = triangulateProcessNoise(dtPred);
            [xPred,pPred] = extrapolateEstimate(x0,p0,phiPred,qPred);
            H = triangulateMeasurementMatrix(xPred);
            [zPred,rPred] = predictTriangulateMeasurement(xPred,pPred,H);

            [xFilt,pFilt] = filterEstimate(xPred,pPred,z,zPred,r,rPred,H);
        else
            xFilt = x0;  pFilt = p0;
        end
        
        phiExtrap = triangulateStateTransitionMatrix(dtExtrap);
        qExtrap = triangulateProcessNoise(dtExtrap);
        [xExtrap,pExtrap] = extrapolateEstimate(xFilt,pFilt,phiExtrap,qExtrap);
        
    otherwise
        disp('ERROR: commonEkf: Invalid sensorType input');
        return;
end

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

% Unscented Kalman filter
%
% Predicted at measurement:
% 
% Filtering:
%

end

%--------------------------------------------------------------------------
% radarInit
%--------------------------------------------------------------------------
function [x0,p0] = radarInit(z,r)

    x0(1) = z(2)*cos(z(1));
    x0(2) = z(2)*sin(z(1));
    x0(3) = z(3)*cos(z(1))-z(2)*sin(z(1));
    x0(4) = z(3)*sin(z(1))+z(2)*cos(z(1));
    
    J = zeros(4,3);
    J(1,1) = -z(2)*sin(z(1));
    J(1,2) = cos(z(1));
    J(2,1) = z(2)*cos(z(1));
    J(2,2) = sin(z(1));
    J(3,1) = -z(3)*sin(z(1))-z(2)*cos(z(1));
    J(3,2) = -sin(z(1));
    J(3,3) = cos(z(1));
    J(4,1) = z(3)*cos(z(1))-z(2)*sin(z(1));
    J(4,2) = cos(z(1));
    J(4,3) = sin(z(1));
    
    p0 = J*r*J';

end

%--------------------------------------------------------------------------
% triangulateInit
%--------------------------------------------------------------------------
function [x0,p0] = triangulateInit(z,r,d)



end

%--------------------------------------------------------------------------
% radarStateTransitionMatrix
%--------------------------------------------------------------------------
function [phi] = radarStateTransitionMatrix(dt)

    phi = eye(4);
    rateBand = dt.*eye(2);
    phi(1:2,3:4) = rateBand;

end

%--------------------------------------------------------------------------
% triangulateStateTransitionMatrix
%--------------------------------------------------------------------------
function [phi] = triangulateStateTransitionMatrix(dt)

    phi = eye(8);
    rateBand = dt.*eye(2);
    phi(1:2,3:4) = rateBand;
    phi(5:6,7:8) = rateBand;

end

%--------------------------------------------------------------------------
% radarProcessNoise
%--------------------------------------------------------------------------
function [q] = radarProcessNoise(dt)
   
    G = [dt^2/2;dt^2/2;dt;dt];
    q = G*G'*0.01;

end

%--------------------------------------------------------------------------
% triangulateProcessNoise
%--------------------------------------------------------------------------
function [q] = triangulateProcessNoise(dt)

    q = zeros(8);

end

%--------------------------------------------------------------------------
% extrapolateEstimate
%--------------------------------------------------------------------------
function [xExtrap,pExtrap] = extrapolateEstimate(x,p,phi,q)

    xExtrap = phi*x;
    pExtrap = phi*p*phi'+q;

end

%--------------------------------------------------------------------------
% radarMeasurementMatrix
%--------------------------------------------------------------------------
function [H] = radarMeasurementMatrix(x)

    H = zeros(3,4);
    r = sqrt(x(1)^2+x(2)^2);
    r2 = r^2;
    r3 = r^3;
    
    H(1,1) = -x(2)/r2;
    H(1,2) = x(1)/r2;
    H(2,1) = x(1)/r;
    H(2,2) = x(2)/r;
    H(3,1) = (x(3)*r^2 - x(3)*x(1)^2 - x(4)*x(2)*x(1))/r3;
    H(3,2) = (x(4)*r^2 - x(4)*x(2)^2 - x(3)*x(1)*x(2))/r3;
    H(3,3) = H(2,1);
    H(3,4) = H(2,2);

end

%--------------------------------------------------------------------------
% triangulateMeasurementMatrix
%--------------------------------------------------------------------------
% function [H] = triangulateMeasurementMatrix(x)
% 
%     H = zeros(4,8);
%     r1 = sqrt(x(1)^2+x(2)^2);  r12 = r1^2;  r14 = r12^2;
%     r2 = sqrt(x(5)^2+x(6)^2);  r22 = r2^2;  r24 = r22^2;
%     
%     H(1,1) = -x(2)/r12;
%     H(1,2) = x(1)/r12;
%     H(2,1) = (-x(4)*x(1)^2+2*x(3)*x(1)*x(2)+x(4)*x(2)^2)/r14;
%     H(2,2) = -(x(3)*x(1)^2+2*x(4)*x(1)*x(2)-x(3)*x(2)^2)/r14;
%     H(2,3) = H(1,1);
%     H(2,4) = H(1,2);
%  
%     H(3,5) = -x(6)/r22;
%     H(3,6) = x(5)/r22;
%     H(4,5) = (-x(8)*x(5)^2+2*x(7)*x(5)*x(6)+x(8)*x(6)^2)/r24;
%     H(4,6) = -(x(7)*x(5)^2+2*x(8)*x(5)*x(6)-x(7)*x(6)^2)/r24;
%     H(4,7) = H(3,5);
%     H(4,8) = H(3,6);
% 
% end

%--------------------------------------------------------------------------
% predictRadarMeasurement
%--------------------------------------------------------------------------
function [zPred,rPred] = predictRadarMeasurement(x,p,H)

    az = atan2(x(2),x(1));
    r = sqrt(x(1)^2+x(2)^2);
    rDot = (x(3)*x(1)+x(4)*x(2))/r;

    zPred = [az;r;rDot];
    rPred = H*p*H';

end

%--------------------------------------------------------------------------
% predictTriangulateMeasurement
%--------------------------------------------------------------------------
function [zPred,rPred] = predictTriangulateMeasurement(x,p,H)

    az1 = atan2(x(2),x(1));
    az2 = atan2(x(5),x(6));
    az1Dot = sqrt(x(1)^2+x(2)^2);
    az2Dot = (x(3)*x(1)+x(4)*x(2))/r;

    zPred = [az1;az1Dot;az2;az2Dot];
    rPred = H*p*H';

end

%--------------------------------------------------------------------------
% filterEstimate
%--------------------------------------------------------------------------
function [xFilt,pFilt] = filterEstimate(x,p,z,zPred,r,rPred,H)

    y = z-zPred;
    S = r+rPred;
    K = p*H'*inv(S);
    xFilt = x+K*y;
    pFilt = (eye(length(x))-K*H)*p;

end
