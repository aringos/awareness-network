classdef scn_RaspPi1080p6M12_LoRa
    
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

        function scenario = scn_RaspPi1080p6M12_LoRa(dt, tend)
            scenario.sensors = ...
              [Sensor('Pi_6mmM12', [-20;100], 290*pi/180); 
               Sensor('Pi_6mmM12', [20;180], -110*pi/180);
               Sensor('Pi_6mmM12', [-20;260], 290*pi/180);
               Sensor('Pi_6mmM12', [20;340], -110*pi/180);
               Sensor('Pi_6mmM12', [-20;420], 290*pi/180);
               Sensor('Pi_6mmM12', [20;500], -110*pi/180);
               Sensor('Pi_6mmM12', [-20;580], 290*pi/180);
               Sensor('Pi_6mmM12', [20;660], -110*pi/180);
               Sensor('Pi_6mmM12', [-20;740], 290*pi/180);
               Sensor('Pi_6mmM12', [20;820], -110*pi/180);
               Sensor('Pi_6mmM12', [-20;860], 290*pi/180)];
            
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

