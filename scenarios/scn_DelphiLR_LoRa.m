classdef scn_DelphiLR_LoRa
    
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

        function scenario = scn_DelphiLR_LoRa(dt, tend)
            scenario.sensors = ...
               [Sensor('Delphi_Long_ESR', [20;170], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;265], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;360], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;455], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;550], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;645], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;740], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;835], 257*pi/180);...
                Sensor('Delphi_Long_ESR', [20;930], 257*pi/180)];
            
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

