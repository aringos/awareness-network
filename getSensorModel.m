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
%--------------------------------------------------------------------------

function [ sensor ] = getSensorModel( modelName, position, az, el, objWidth )
    nPixelsForDetection = 3;

    sensor.H    = [0;0;0;0];
    sensor.max  = [0;0;0;0];
    sensor.pos  = position;
    sensor.TI2B = eul2rotm([az -el 0]);
    sensor.TB2I = sensor.TI2B';
    sensor.R    = -1*ones(4,4);
    sensor.dt   = 0;
    
    switch modelName
        case 'Delphi_Mid_ESR'
            sensor.H   = [1;1;1];
            sensor.max = [45*pi/180; 60; 9999];
            sensor.R   = [0.5*pi/180 0 0; 0 0 0.25; 0 0 0.12];
        case 'Velodyne_VLP16'
            sensor.H   = [1;1;1;0];
            sensor.max = [180*pi/180; 15*pi/180; 100; 0];
            sensor.R   = [0.05 0 0 0; 0 1.0 0 0; 0 0 0.03 0; 0 0 0 0];
        case 'R20A'
            sensor.H   = [1;1;0;0];
            sensor.R   = [0.094*pi/180 0 0 0; 0 0.094*pi/180 0 0; 0 0 0 0; 0 0 0 0];
            pxRange    = objWidth/(2*atan(sensor.R(1,1)*nPixelsForDetection/2)); 
            sensor.max = [30*pi/180; 30*pi/180; pxRange; 0];
    end  
    
    %1-sigma to variance
    sensor.R = sensor.R.^2;
end



                
                
                
                
                
                
                