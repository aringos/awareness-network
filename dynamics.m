%--------------------------------------------------------------------------
%
% dynamics
%
% Austin Smith
% SIE-554A
% 9/29/2017
%
% Models simple 3-D kinematic system dynamics with variable time step
%
% Inputs:  state vector (x=[x,y,z,vx,vy,vz,ax,ay,az]), time step (dt)          
%
% Outputs: propagated state vector (xBar)
%
%--------------------------------------------------------------------------

function [ xBar ] = dynamics( x, dt )

% Check for 9-D state vector
if length(x) ~= 9
    disp('ERROR: state vector must contain 9 states!');
    return
end

% Form Newtonian plant matrix for 3-D kinematics
A = [ 1, 0, 0, dt,  0,  0, 0.5*dt^2,        0,        0; ...
      0, 1, 0,  0, dt,  0,        0, 0.5*dt^2,        0; ...
      0, 0, 1,  0,  0, dt,        0,        0, 0.5*dt^2; ...
      0, 0, 0,  1,  0,  0,       dt,        0,        0; ...
      0, 0, 0,  0,  1,  0,        0,       dt,        0; ...
      0, 0, 0,  0,  0,  1,        0,        0,       dt; ...
      0, 0, 0,  0,  0,  0,        1,        0,        0; ...
      0, 0, 0,  0,  0,  0,        0,        1,        0; ...
      0, 0, 0,  0,  0,  0,        0,        0,        1 ]; ...

% Propagate state from x(t0) to x(t0+dt)
xBar = A*x;

end

