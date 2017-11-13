%--------------------------------------------------------------------------
%
% dynamics
%
% Austin Smith
% SIE-554A
% 9/29/2017
%
% Models simple 2-D kinematic system dynamics with variable time step
%
% Inputs:  state vector (x=[x,y,vx,vy,ax,ay]), time step (dt)          
%
% Outputs: propagated state vector (xBar)
%
%--------------------------------------------------------------------------

function [ xBar ] = dynamics( x, dt )

% Check for 6-D state vector
if length(x) ~= 6
    disp('ERROR: state vector must contain 6 states!');
    return
end

% Form Newtonian plant matrix for 2-D kinematics
A = [ 1, 0, dt,  0, 0.5*dt^2,        0; ...
      0, 1,  0, dt,        0, 0.5*dt^2; ...
      0, 0,  1,  0,       dt,        0; ...
      0, 0,  0,  1,        0,       dt; ...
      0, 0,  0,  0         1,        0; ...
      0, 0,  0,  0,        0,        1]; ...

% Propagate state from x(t0) to x(t0+dt)
xBar = A*x;

end

