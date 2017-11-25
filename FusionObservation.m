classdef FusionObservation

    
    properties
        t          = 0;
        z          = 0;
        R          = [];
        sensor_pos = [];
        type       = 'NA';
    end
    
    methods
        function obs = FusionObservation(z, R, t, pos)
            obs.t          = t;
            obs.z          = [z(1); z(3); z(4)];
            obs.R          = diag([R(1,1); R(3,3); R(4,4)]);
            obs.sensor_pos = pos;  
        end
        
    end
    
end

