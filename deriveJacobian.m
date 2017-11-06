%--------------------------------------------------------------------------
%
% deriveJacobian
%
% Austin Smith
% SIE-554A
% 11/04/2017
%
% Uses symbolic toolbox to compute Jacobians for EKF measurement matrices
%
%--------------------------------------------------------------------------

% 2-D Radar
syms az r rDot;
syms x y vx vy;

az    = atan(y/x);
r     = sqrt(x^2+y^2);
rDot  = (vx*x+vy*y)/r;

dPol_dCart = jacobian([az,r,rDot],[x,y,vx,vy]);
disp('Radar:');
disp(simplify(dPol_dCart,100));

% 2-D Angle-Only
syms azDot;

az    = atan(y/x);
azDot = 1/(1+(y/x)^2)*(vy*x-y*vx)/x^2;

dAng_dCart = jacobian([az,azDot],[x,y,vx,vy]);
disp('Angle-Only:');
disp(simplify(dAng_dCart,100));

