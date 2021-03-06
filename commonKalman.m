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
% to the sensor(s). Triangulate outputs filtered states in measurement
% format and extrapolated states in local NED format.
%
% Inputs:  current filtered state (x0=[x,y,vx,vy] or [az1,az1Dot,az2,az2Dot])
%          current filtered error (diag(p0)=[xVar,yVar,vxVar,vyVar] or
%                                           [az1Var,az1DotVar,az2Var,az2DotVar])
%          sensor2 position rel. sensor1 (d=[x21,y21])
%          observation vector (z=[meas1,meas2,meas3,...])
%          observation noise (diag(r)=[var1,var2,var3,...])
%          time vector (time=[tObsPrev,tObs,tEst])
%              where the times are: prev. observation, current, and desired
%              filter estimate time
%
% Outputs: new filtered state (xHat=[x,y,vx,vy] or [az1,az1Dot,az2,az2Dot])
%          new filtered error (diag(pHat)=[xVar,yVar,vxVar,vyVar] or 
%                                         [az1Var,az1DotVar,az2Var,az2DotVar])
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
elseif length(z)==4
    sensorType = 1;
else
    sensorType = 99;
end

switch sensorType
    
    % Radar sensor
    case 0
        if init==1
            [xFilt,pFilt] = radarInit(z,r);
            xExtrap = xFilt; 
            pExtrap = pFilt;
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
            xFilt = x0;  
            pFilt = p0;
        end
        
        phiExtrap = radarStateTransitionMatrix(dtExtrap);
        qExtrap = radarProcessNoise(dtExtrap);
        [xExtrap,pExtrap] = extrapolateEstimate(xFilt,pFilt,phiExtrap,qExtrap);
    
    % Two passive sensors
    case 1
        if init==1
           xFilt = z;  pFilt = r;
           [xExtrap,pExtrap] = triangulate(xFilt,pFilt,d);
           return;
        end
        
        if update==1
            phiPred = triangulateStateTransitionMatrix(dtPred);
            qPred = triangulateProcessNoise(dtPred);
            [xPred,pPred] = extrapolateEstimate(x0,p0,phiPred,qPred);
            %H = triangulateMeasurementMatrix(xPred,d);
            H = eye(4);
            %[zPred,rPred] = predictTriangulateMeasurement(xPred,pPred,d);
            zPred = xPred;  rPred = pPred;
            [xFilt,pFilt] = filterEstimate(xPred,pPred,z,zPred,r,rPred,H);
            %pxz = triangulateCrossCovariance(xPred,pPred,zPred,rPred);
            %[xFilt,pFilt] = unscentedFilterEstimate(xPred,pPred,z,zPred,r,rPred,pxz);
        else
            xFilt = x0;  pFilt = p0;
        end
        phiExtrap = triangulateStateTransitionMatrix(dtExtrap);
        qExtrap = triangulateProcessNoise(dtExtrap);
        [xExtrap,pExtrap] = extrapolateEstimate(xFilt,pFilt,phiExtrap,qExtrap);
        [xExtrap,pExtrap] = triangulate(xExtrap,pExtrap,d);
        
    otherwise
        disp('ERROR: commonKalman: Invalid sensorType input');
        return;
end

end

%--------------------------------------------------------------------------
% radarInit
%--------------------------------------------------------------------------
function [x0,p0] = radarInit(z,r)

    azDot0 = 0.0;
    x0(1) = z(2)*cos(z(1));
    x0(2) = z(2)*sin(z(1));
    x0(3) = z(3)*cos(z(1))-z(2)*sin(z(1))*azDot0;
    x0(4) = z(3)*sin(z(1))+z(2)*cos(z(1))*azDot0;
    
    J = zeros(4,3);
    J(1,1) = -z(2)*sin(z(1));
    J(1,2) = cos(z(1));
    J(2,1) = z(2)*cos(z(1));
    J(2,2) = sin(z(1));
    J(3,1) = -z(3)*sin(z(1))-z(2)*cos(z(1))*azDot0;
    J(3,2) = -sin(z(1))*azDot0;
    J(3,3) = cos(z(1));
    J(4,1) = z(3)*cos(z(1))-z(2)*sin(z(1))*azDot0;
    J(4,2) = cos(z(1))*azDot0;
    J(4,3) = sin(z(1));
    
    p0 = J*r*J';

end

%--------------------------------------------------------------------------
% triangulateMeasurementToState
%--------------------------------------------------------------------------
function [x] = triangulateMeasurementToState(z,d)

    u1Xu2 = cos(z(1))*sin(z(3))-sin(z(1))*cos(z(3));
    rng1 = -(d(1)*sin(z(3))+(d(2)*cos(z(3))))/abs(u1Xu2);
    rng2 =  (d(1)*sin(z(1))-(d(2)*cos(z(1))))/abs(u1Xu2);

    u1Xu2Dot = sign(u1Xu2)*(-sin(z(1))*z(2)*sin(z(3))+cos(z(1))*cos(z(3))*z(4) ...
               -cos(z(1))*z(2)*cos(z(3))+sin(z(1))*sin(z(3))*z(4));
    rng1Dot = sign(rng1)*((-d(1)*cos(z(3))*z(4)-d(2)*sin(z(3))*z(4))*abs(u1Xu2) ...
               -(-d(1)*sin(z(3))+d(2)*cos(z(3)))*u1Xu2Dot)/u1Xu2^2;
    rng2Dot = sign(rng2)*((d(1)*cos(z(1))*z(2)+d(2)*sin(z(1))*z(2))*abs(u1Xu2) ...
               -(d(1)*sin(z(1))-d(2)*cos(z(1)))*u1Xu2Dot)/u1Xu2^2;
    
    x(1) = abs(rng1)*cos(z(1));
    x(2) = abs(rng1)*sin(z(1));
    x(3) = rng1Dot*cos(z(1))-abs(rng1)*sin(z(1))*z(2);
    x(4) = rng1Dot*sin(z(1))+abs(rng1)*cos(z(1))*z(2);
    x(5) = abs(rng2)*cos(z(3));
    x(6) = abs(rng2)*sin(z(3));
    x(7) = rng2Dot*cos(z(3))-abs(rng2)*sin(z(3))*z(4);
    x(8) = rng2Dot*sin(z(3))+abs(rng2)*cos(z(3))*z(4);

end

%--------------------------------------------------------------------------
% triangulate
%--------------------------------------------------------------------------
function [x0,p0] = triangulate(z,r,d)

    x0 = triangulateMeasurementToState(z,d);
    
    % Use unscented transform to get covariance matrix
    n = length(z);
    p0 = zeros(length(x0));
    W0 = -0.25;
    sig = sqrtm(n/(1-W0)*r);
    for i=1:(2*n)
        sigPt = sign(i-n+0.01)*sig(:,ceil(i/2));
        xPt = triangulateMeasurementToState(z+sigPt,d);
        p0 = p0+(x0-xPt)*(x0-xPt)';
    end
    p0 = p0*(1-W0)/(2*n);
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

    phi = eye(4);
    phi(1,2) = dt;
    phi(3,4) = dt;

end

%--------------------------------------------------------------------------
% radarProcessNoise
%--------------------------------------------------------------------------
function [q] = radarProcessNoise(dt)
   
    G = [dt^2/2;dt^2/2;dt;dt];
    q = G*G'*0.75;

end

%--------------------------------------------------------------------------
% triangulateProcessNoise
%--------------------------------------------------------------------------
function [q] = triangulateProcessNoise(dt)

    G = [dt^2/2;dt;dt^2/2;dt];
    q = G*G'*0.002;

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
%  function [H] = triangulateMeasurementMatrix(x,d)
%  
%     H = zeros(4,8);
% 
%     sign1 = sign(d(1)*x(2)-d(2)*x(1));
%     sign2 = sign(-d(1)*x(6)+d(2)*x(5));
%     signD = sign(x(1)*x(6)-x(2)*x(5));
%     dx1 = sign1*signD*d(1);  dy1 = sign1*signD*d(2);
%     dx2 = sign2*signD*d(1);  dy2 = sign2*signD*d(2);
%     rho1  = (sign1*d(1)+signD*x(1))^2+(sign1*d(2)+signD*x(2))^2;
%     rho2  = (sign2*d(1)+signD*x(5))^2+(sign2*d(2)+signD*x(6))^2;
% %     dx1 = d(1);  dy1 = d(2);
% %     dx2 = d(1);  dy2 = d(2);
% %     rho1  = (d(1)+x(1))^2+(d(2)+x(2))^2;
% %     rho2  = (d(1)+x(5))^2+(d(2)+x(6))^2;
%     
%     H(1,5) = -(dy1+x(6))/rho2;
%     H(1,6) =  (dx1+x(5))/rho2;
%     H(2,5) =  x(8)/rho2-((x(8)*(dx1+x(5))-x(7)*(dy1+x(6)))*(2*dx1+2*x(5)))/rho2^2;
%     H(2,6) = -x(7)/rho2-((x(8)*(dx1+x(5))-x(7)*(dy1+x(6)))*(2*dy1+2*x(6)))/rho2^2;
%     H(2,7) = H(1,5);
%     H(2,8) = H(1,6);
%     H(3,1) = -(dy2+x(2))/rho1;
%     H(3,2) =  (dx2+x(1))/rho1;
%     H(4,1) =  x(4)/rho1-((x(4)*(dx2+x(1))-x(3)*(dy2+x(2)))*(2*dx2+2*x(1)))/rho1^2;
%     H(4,2) = -x(3)/rho1-((x(4)*(dx2+x(1))-x(3)*(dy2+x(2)))*(2*dy2+2*x(2)))/rho1^2;
%     H(4,3) = H(3,1);
%     H(4,4) = H(3,2);
%     
%  end

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
% triangulateStateToMeasurement
%--------------------------------------------------------------------------
function [z] = triangulateStateToMeasurement(x,d)

    sign1 = sign(d(1)*x(2)-d(2)*x(1));
    sign2 = sign(-d(1)*x(6)+d(2)*x(5));
    signD = sign(x(1)*x(6)-x(2)*x(5));
    dx1 = sign1*signD*d(1);  dy1 = sign1*signD*d(2);
    dx2 = sign2*signD*d(1);  dy2 = sign2*signD*d(2);
    rho1  = (sign1*d(1)+signD*x(1))^2+(sign1*d(2)+signD*x(2))^2;
    rho2  = (sign2*d(1)+signD*x(5))^2+(sign2*d(2)+signD*x(6))^2;
%     dx1 = d(1);  dy1 = d(2);
%     dx2 = d(1);  dy2 = d(2);
%     rho1  = (d(1)+x(1))^2+(d(2)+x(2))^2;
%     rho2  = (d(1)+x(5))^2+(d(2)+x(6))^2;

    z(1) = atan2((x(6)+dy1),(x(5)+dx1));
    z(3) = atan2((x(2)+dy2),(x(1)+dx2));
    z(2) = (x(8)*(x(5)+dx1)-x(7)*(x(6)+dy1))/rho2^2;
    z(4) = (x(4)*(x(1)+dx2)-x(3)*(x(2)+dy2))/rho1^2;

end

%--------------------------------------------------------------------------
% predictTriangulateMeasurement
%--------------------------------------------------------------------------
function [zPred,rPred] = predictTriangulateMeasurement(x,p,d)

    zPred = triangulateStateToMeasurement(x,d);
    
    % Use unscented transform to get covariance matrix
    n = length(x);
    rPred = zeros(length(zPred));
    W0 = 0;
    sig = sqrtm(n/(1-W0)*p);
    for i=1:(2*n)
        sigPt = sign(i-n+0.01)*sig(:,ceil(i/2));
        zPt = triangulateStateToMeasurement(x+sigPt,d);
        rPred = rPred+(zPred-zPt)*(zPred-zPt)';
    end
    rPred = rPred*(1-W0)/(2*n);

end

%--------------------------------------------------------------------------
% triangulateCrossCovariance
%--------------------------------------------------------------------------
function [pxz] = triangulateCrossCovariance(x,p,z,r)

    % Use unscented transform to get covariance matrix
    nx = length(x);
    nz = length(z);
    pxz= zeros(nx,nz);
    W0 = 0;
    sigX = sqrtm(n/(1-W0)*p);
    sigZ = sqrtm(n/(1-W0)*r);
    for i=1:(2*n)
        xPt = sign(i-n+0.01)*sigX(:,ceil(i/2));
        zPt = sign(i-n+0.01)*sigZ(:,ceil(i/2));
        pxz = pxz+(x-xPt)*(z-zPt)';
    end
    pxz = pxz*(1-W0)/(2*n);

end

%--------------------------------------------------------------------------
% unscentedFilterEstimate
%--------------------------------------------------------------------------
function [xFilt,pFilt] = unscentedFilterEstimate(x,p,z,zPred,r,rPred,pxz)

    y = z-zPred;
    S = r+rPred;
    K = pxz*inv(S);
    xFilt = x+K*y;
    pFilt = p-K*S*K';

end

%--------------------------------------------------------------------------
% filterEstimate
%--------------------------------------------------------------------------
function [xFilt,pFilt] = filterEstimate(x,p,z,zPred,r,rPred,H)

    y = z-zPred;
    conditioner = max(diag(r))*eye(length(z));
    S = (r+rPred)*conditioner;
    K = p*H'*inv(S)*conditioner;
    xFilt = x+K*y;
    pFilt = (eye(length(x))-K*H)*p;

end
