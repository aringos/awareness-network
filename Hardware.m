classdef Hardware
    
    properties
        costPerUnit        = 0;
        powerDrawPerUnit_W = 0;
        miscCosts          = 25.0;
    end
    
    methods
        
        function hw = Hardware(modelName)
            switch modelName
                case 'Pi_Zero'
                    hw.costPerUnit        = 10.0;     
                    hw.powerDrawPerUnit_W = 0.9;
                case 'Pi_3'
                    hw.costPerUnit        = 35.00;     
                    hw.powerDrawPerUnit_W = 6.5;
            end
        end
        
    end
    
end

