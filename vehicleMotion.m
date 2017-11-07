%--------------------------------------------------------------------------
%
% vehicleMotion
%
% Austin Smith
% SIE-554A
% 9/29/2017
%
% Generates NED vehicle motion models
%
% Inputs:  desired vehicle motion model (model), time step (dt), final time (tf)
%
% Outputs: 3xn acceleration command vector vs. time (accelCommands)
%
%--------------------------------------------------------------------------

function [ accelCommands ] = vehicleMotion( model, dt, tf )

% Unit conversions
mph2mps = unitsratio('m','mi')/3600;

% Generate time vector
t = 0:dt:tf;
n = length(t);

% Set acceleration command vector
switch model
    
    % Vehicle cruising at constant speed
    case 'cruise'
        accelCommands = zeros(2,n);
    
    % Car turning N->E onto another city street
    % - Lane width is 3.7m and corner radius is about another 4m, therefore
    %   assume the turning radius is about 3.7/2+4=5.85m.
    % - Turning distance is simply R*(pi/2) for a 90 deg turn.
    % - It is assumed that the turn begins at t0, but this can change.
    case 'carTurnCity'
        speed            = 20*mph2mps;
        accelCentripetal = speed^2/5.85;
        turningDistance  = 5.85*(pi/2);
        turningTime      = turningDistance/speed;
        turningRate      = (pi/2)/turningTime;
        accelCommands    = accelCentripetal.* ...
                               [-sin(turningRate*t).*(t < turningTime); ...
                                 cos(turningRate*t).*(t < turningTime)];
    
    % Quadcopter takeoff
    % - Takeoff will have the highest altitude rate change of any typical
    %   quadcopter maneuver, making it most difficult to track.
    % - Typical high altitude rates are < 40 m/s for climb.
    % - Typical high altitude accelerations are < 35 m/s^2.
    % - There is no target altitude in this model.
    case 'quadcopterTakeoff'
        altitudeAccel = 19.6; %2 G's
        altitudeRate  = 25;
        accelTime     = altitudeRate/altitudeAccel;
        accelCommands = [zeros(1,n); ...
                         -altitudeAccel*( t < accelTime )];
        
    otherwise
        accelCommands = zeros(2,n);    
end


end

