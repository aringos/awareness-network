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

% 2-D Triangulate
syms az1 az2 az1Dot az2Dot x1 y1 vx1 vy1 x2 y2 vx2 vy2 dx dy;

az1    = atan((y2+dy)/(x2+dx));
az2    = atan((y1+dy)/(x1+dx));
az1Dot = (vy2*(x2+dx)-vx2*(y2+dy))/((x2+dx)^2+(y2+dy)^2);
az2Dot = (vy1*(x1+dx)-vx1*(y1+dy))/((x1+dx)^2+(y1+dy)^2);

dTri_dCart = jacobian([az1,az1Dot,az2,az2Dot],[x1,y1,vx1,vy1,x2,y2,vx2,vy2]);
disp('Triangulation:');
disp(simplify(dTri_dCart,100));

clear('az1','az1Dot','az2','az2Dot');
syms az1 az1Dot az2 az2Dot;
syms r1 r1Dot r2 r2Dot;
r1    = (-dx*sin(az2)+dy*cos(az2))/(cos(az1)*sin(az2)-sin(az1)*cos(az2));
r2    = (dx*sin(az1)-dy*cos(az1))/(cos(az1)*sin(az2)-sin(az1)*cos(az2));
r1Dot = ((-dx*cos(az2)*az2Dot-dy*sin(az2)*az2Dot)*(cos(az1)*sin(az2)-sin(az1)*cos(az2))...
        -(-dx*sin(az2)+dy*cos(az2))*(-sin(az1)*az1Dot*sin(az2)+cos(az1)*cos(az2)*az2Dot...
          -cos(az1)*az1Dot*cos(az2)+sin(az1)*sin(az2)*az2Dot))/...
        (cos(az1)*sin(az2)-sin(az1)*cos(az2))^2;
r2Dot = ((dx*cos(az1)*az1Dot+dy*sin(az1)*az1Dot)*(cos(az1)*sin(az2)-sin(az1)*cos(az2))...
        -(dx*sin(az1)-dy*cos(az1))*(-sin(az1)*az1Dot*sin(az2)+cos(az1)*cos(az2)*az2Dot...
          -cos(az1)*az1Dot*cos(az2)+sin(az1)*sin(az2)*az2Dot))/...
        (cos(az1)*sin(az2)-sin(az1)*cos(az2))^2;
    
dTriR_dAz = jacobian([r1,r1Dot,r2,r2Dot],[az1,az1Dot,az2,az2Dot]);
disp('Triangulation-Polar:');
disp(simplify(dTriR_dAz,100));
