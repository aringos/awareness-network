classdef scn_OV7670_LoRa
    
    properties
        sensors = [];
        network = 0;
        hardware = 0;
        fusion = 0;
        vehicleAccel = [];
        vehicleInitD = zeros(2,1);
        vehicleInitV = zeros(2,1);
    end
    
    methods

        function scenario = scn_OV7670_LoRa(dt, tend)
            scenario.sensors = ...
               [Sensor('Arducam_OV7670', [20;65],   257*pi/180);...
                Sensor('Arducam_OV7670', [-20;130], -77*pi/180);...
                Sensor('Arducam_OV7670', [20;195],  257*pi/180);...
                Sensor('Arducam_OV7670', [-20;260], -77*pi/180);...
                Sensor('Arducam_OV7670', [20;325],  257*pi/180);...
                Sensor('Arducam_OV7670', [-20;390], -77*pi/180);...
                Sensor('Arducam_OV7670', [20;455],  257*pi/180);...
                Sensor('Arducam_OV7670', [-20;520], -77*pi/180);...
                Sensor('Arducam_OV7670', [20;585],  257*pi/180);...
                Sensor('Arducam_OV7670', [-20;650], -77*pi/180);...
                Sensor('Arducam_OV7670', [20;715],  257*pi/180);...
                Sensor('Arducam_OV7670', [-20;780], -77*pi/180);...
                Sensor('Arducam_OV7670', [20;845],  257*pi/180)];
            
            scenario.network      = Network('LoRa_RN2483A');
            scenario.hardware     = Hardware('Pi_3');
            scenario.fusion       = FusionCenter();  
            
            mph2mps = unitsratio('m','mi')/3600;
            scenario.vehicleAccel = vehicleMotion( 'cruise', dt, tend ); 
            scenario.vehicleInitD = [0;0];
            scenario.vehicleInitV = [0;35*mph2mps];
        end
     
    end
    
end

