classdef FusionCenter
    
    properties
        x = zeros(6,1);
        P = eye(6,6);
        t = -1.0;
        initialized = 0;
    end
    
    methods
        function kalman = FusionCenter(kalman)
           kalman.initialized = 0;
        end
        
        function x = transformZtoX(kalman, z)
           x = [z(2)*cos(z(1));
                z(2)*sin(z(1));
                z(3)*cos(z(1))-z(2)*sin(z(1))*azDot0;
                z(3)*sin(z(1))+z(2)*cos(z(1))*azDot0];
        end
        
        function J = getJacobian(kalman, azRate, z)
           J = zeros(4,3);
           J(1,1) = -z(2)*sin(z(1));
           J(1,2) = cos(z(1));
           J(2,1) = z(2)*cos(z(1));
           J(2,2) = sin(z(1));
           J(3,1) = -z(3)*sin(z(1))-z(2)*cos(z(1))*azDot0;
           J(3,2) = -sin(z(1))*azDot0;
           J(3,3) = cos(z(1));
           J(4,1) = z(3)*cos(z(1))-z(2)*sin(z(1))*azDot0;
           J(4,2) = cos(z(1))*azDot0;
           J(4,3) = sin(z(1));        
        end
        
        function kalman = initialize(kalman, z, R, t)
           azRate   = 0.01;
           kalman.x = kalman.transformZtoX(z); 
           J        = kalman.getJacobian(azRate, z);
           kalman.P = J*R*J';
           kalman.initialized = 1;
        end
        
        function kalman = measUpdate(kalman, z, R, t) 
            
        end
        
        function kalman = update(kalman, observationPacket)
           z = observationPacket.observation_x;
           R = observationPacket.observation_P;
           t = observationPacket.observation_t;
           if ~kalman.initialized
              kalman = kalman.initialize(z,R,t);
           else
              kalman = kalman.measUpdate(z,R,t); 
           end
        end
        
    end
    
end

