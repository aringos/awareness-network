%--------------------------------------------------------------------------
%
% getSensorModel
%
% Anthony Rodriguez
% SIE-554A
% 10/5/2017
%
% Get a specific sensor model structure from parameterization list
%
% Inputs:  Name of sensor to model (modelName)
%          Column position vector of sensor in NED
%          Sensor aiming azimuth from due North (radians)
%          Sensor aiming elevation from due North (radians)
%          objWidth is width of object in meters (used for pixel maxrange)
%
% Outputs: Sensor structure for use in producing measurements
%          H = [az,azDot,range,rDot];
%--------------------------------------------------------------------------

function [ sensor ] = getSensorModel( modelName, position, az, objWidth )

    if (length(position)>2)
        disp('ERROR: getSensorModel() position vector must be of length 2');
        return
    end

    nPixelsForDetection = 3;

    sensor.H        = zeros(4,1);
    sensor.max      = zeros(4,1);
    sensor.pos      = position;
    sensor.TB2I     = [cos(az),-sin(az);sin(az),cos(az)];
    sensor.TI2B     = sensor.TB2I';
    sensor.R        = -1*ones(4);
    sensor.dt       = 0;
    sensor.color    = rand(3,1).*.7;
    sensor.z        = [];
    sensor.z_truth  = [];
    sensor.lastObservationTime = -10.0;
    
    switch modelName
        case 'Delphi_Mid_ESR'
            sensor.H   = [1;0;1;1];
            sensor.max = [45*pi/180; 0; 60; 9999];
            sensor.R   = [0.5*pi/180 0 0; 0 0.25 0; 0 0 0.12];
            sensor.dt  = 50e-3;
        case 'Velodyne_VLP16'
            sensor.H   = [1;0;1;0];
            sensor.max = [180*pi/180; 0; 100; 0];
            sensor.R   = [0.05 0 0; 0 0.03 0];
            %sensor.dt  = ???
        case 'R20A'
            sensor.H   = [1;1;0;0];
            sensor.R   = [0.094*pi/180 0; 0 0.0094*pi/180];
            sensor.dt  = 1/30;
            pxRange    = objWidth/(2*atan(sensor.R(1,1)*nPixelsForDetection/2)); 
            sensor.max = [30*pi/180; 0.5*30*pi/180/sensor.dt; pxRange; 0];
    end  
    
    %1-sigma to variance
    sensor.R = sensor.R.^2;
end



                
                
                
                
                
                
                