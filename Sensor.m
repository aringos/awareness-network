classdef Sensor
    
    properties
        nPixelsForDetection = 3;
        H        = zeros(4,1);
        max      = zeros(4,1);
        R        = -1*ones(4);
        dt       = 0;
        color    = rand(3,1).*.7;
        z        = [];
        z_truth  = [];
        filter   = SphericalFilter
        TB2I     = [];
        TI2B     = [];
        pos      = [0;0];
        tracking = 0;
        lastObservationTime = -10.0;  
        vertices = [];
        pxRange  = 100;
        cost     = 0;
        
        %Data packet definition
        P_sizeBits           = 0;
        z_sizeBits           = 0;
        tgtClass_sizeBits    = 8;
        obsTime_sizeBits     = 32;
        maxNumObjectsTracked = 30;
        posVar_sizeBits      = 64;
        pos_sizeBits         = 64;
        id_sizeBits          = 32;
        
        z_truth_hist = [];
        z_est_hist = [];
        P_diag_hist = [];
        t_hist = [];
        
    end
    
    methods

        function sensor = Sensor(modelName, position, az, objWidth)
            sensor.TB2I = [cos(az),-sin(az);sin(az),cos(az)];
            sensor.TI2B = sensor.TB2I';  
            sensor.pos  = position;
            nPixelsForDetection = 8;
            switch modelName
                case 'Delphi_Mid_ESR'
                    sensor.P_sizeBits = 32*3*3;
                    sensor.z_sizeBits = 32*3;
                    sensor.H    = [1;0;1;1];
                    sensor.max  = [45*pi/180; 0; 60; 9999];
                    sensor.R    = [0.5*pi/180 0 0 0; 
                                  0 0 0 0; 
                                  0 0 0.25 0;
                                  0 0 0 0.12];
                    sensor.dt   = 50e-3;
                    sensor.cost = 3600.0; 
                case 'Velodyne_VLP16'
                    sensor.P_sizeBits = 32*2*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H    = [1;0;1;0];
                    sensor.max  = [180*pi/180; 0; 100; 0];
                    sensor.R    = [0.05 0 0 0; ...
                                   0 0 0 0; ...
                                   0 0 0.03 0; ...
                                   0 0 0 0];
                    sensor.cost = 8000.0; 
                    %sensor.dt  = ???
                case 'R20A'
                    sensor.P_sizeBits = 32*2*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H   = [1;1;0;0];
                    sensor.R   = [0.094*pi/180 0 0 0; ...
                                  0 0.0094*pi/180 0 0; ...
                                  0 0 0 0; ...
                                  0 0 0 0];
                    sensor.dt  = 1/30;
                    sensor.pxRange = objWidth/(2*atan(sensor.R(1,1)*nPixelsForDetection/2)); 
                    sensor.max = [30*pi/180; 0.5*30*pi/180/sensor.dt; sensor.pxRange; 0];
                case 'Raspberry_Pi_Camera'
                    sensor.H   = [1;1;0;0];
                    sensor.R   = [0.06*pi/180  0           0 0; ...
                                  0            0.06*pi/180/2 0 0; ...
                                  0            0           0 0; ... 
                                  0            0           0 0];
                    sensor.dt  = 1/30;
                    pxRange    = objWidth/(2*atan(sensor.R(1,1)*nPixelsForDetection/2)); 
                    sensor.max = [62*pi/180; 1e9; pxRange; 0];
                case 'TRUTH_CAMERA'
                    sensor.H   = [1;1;0;0];
                    sensor.R   = [1e-3  0    0 0; ...
                                  0     1e-3 0 0; ...
                                  0     0    0 0; ... 
                                  0     0    0 0];
                    sensor.dt  = 1/30;
                    pxRange    = 600; 
                    sensor.max = [pi/2; 1e9; pxRange; 0];
            end  
            %1-sigma to variance
            sensor.R                 = sensor.R.^2;
            sensor                   = sensor.createVertices();
        end

        function sensor = update(sensor, x, t)
            if ( ~sensor.canSeeTarget(x) )
                sensor.tracking = 0; 
                return
            end
            sensor = sensor.getObservations(x);
            if t-sensor.lastObservationTime > sensor.dt 
                if ~sensor.tracking
                   sensor.filter = sensor.filter.initialize(sensor.z, sensor.R, sensor.H, t);
                   sensor.tracking = 1;
                else
                   sensor.filter = sensor.filter.update(sensor.z, sensor.R, t);
                end
                sensor.lastObservationTime = t;
            end
            if ~sensor.tracking
                return
            end
            sensor = sensor.recordTelemetry(t);
        end
        
        function sensor = recordTelemetry(sensor, t)
           sensor.t_hist = [sensor.t_hist t];
           sensor.z_truth_hist = [sensor.z_truth_hist sensor.z_truth];
           [zEst, pEst] = sensor.filter.getExtrapolation(t);
           sensor.z_est_hist = [sensor.z_est_hist, zEst];
           sensor.P_diag_hist = [sensor.P_diag_hist diag(pEst)];
        end
        
        function canSee = canSeeTarget(sensor, x)
            relPos        = [x(1);x(2)]-sensor.pos;
            range         = norm(relPos);
            canSee        = range<sensor.max(3);
            tgtPosInLOS   = sensor.TI2B*relPos;
            tgtLosAzimuth = atan2(tgtPosInLOS(2),tgtPosInLOS(1)); 
            canSee        = canSee && abs(tgtLosAzimuth)<(sensor.max(1))/2.0;
        end
        
        function sensor = getObservations(sensor, x)
            relPos = ([x(1); x(2)]-sensor.pos);
            relVel = [x(3); x(4)];
            range  = norm(relPos);
            rDot   = dot(relVel,relPos./range);
            az     = atan2(relPos(2),relPos(1));
            azDot  = (relPos(1)*relVel(2)-relPos(2)*relVel(1))/range^2;
            z_truth = [az;azDot;range;rDot];
            n   = length(z_truth);
            z = z_truth+randn(n,1).*sqrt(diag(sensor.R));
            z = z.*sensor.H;
            sensor.z_truth = z_truth;
            sensor.z       = z;
        end
        
        function sensor = createVertices(sensor)
            verticesPerSide = 100;
            origin  = sensor.pos;
            range   = sensor.max(3); 
            fov     = sensor.max(1);
            halfFov = fov/2.0;
            leftExtent = [range*cos(halfFov); range*sin(halfFov)];
            leftFovLine = [linspace(0,leftExtent(1),verticesPerSide); ...
                           linspace(0,leftExtent(2),verticesPerSide)];
            rightFovLine = fliplr([leftFovLine(1,:); -leftFovLine(2,:)]);
            leftRadialLine = [];
            for i=1:verticesPerSide/2
                subAngle = i*halfFov/double(verticesPerSide/2);
                leftRadialLine = [leftRadialLine; range*cos(subAngle), range*sin(subAngle)];  
            end
            leftRadialLine   = fliplr(leftRadialLine');
            rightRadialLine  = fliplr([leftRadialLine(1,:); -leftRadialLine(2,:)]);
            thisVertexVector = [leftFovLine leftRadialLine rightRadialLine rightFovLine];
            sensor.vertices  = sensor.TB2I*thisVertexVector; 
            origin           = [origin(1)*ones(1,length(sensor.vertices));...
                                origin(2)*ones(1,length(sensor.vertices))];
            sensor.vertices = origin+sensor.vertices;
        end
        
        function plotTelemetry(sensor)
           if length(sensor.t_hist)==0
              return 
           end
           figure; 
           subplot(4,1,1); grid on; hold on; title('Azimuth');
           plot(sensor.t_hist, sensor.z_truth_hist(1,:).*180/pi, 'g-');
           plot(sensor.t_hist, sensor.z_est_hist(1,:).*180/pi, 'k-');
           plot(sensor.t_hist, sensor.z_est_hist(1,:).*180/pi+sqrt(sensor.P_diag_hist(1,:).*180/pi)*3, 'r--');
           plot(sensor.t_hist, sensor.z_est_hist(1,:).*180/pi-sqrt(sensor.P_diag_hist(1,:).*180/pi)*3, 'r--');
           subplot(4,1,2); grid on; hold on; title('Azimuth Rate');
           plot(sensor.t_hist, sensor.z_truth_hist(2,:).*180/pi, 'g-');
           plot(sensor.t_hist, sensor.z_est_hist(2,:).*180/pi, 'k-');
           plot(sensor.t_hist, sensor.z_est_hist(2,:).*180/pi+sqrt(sensor.P_diag_hist(2,:).*180/pi)*3, 'r--');
           plot(sensor.t_hist, sensor.z_est_hist(2,:).*180/pi-sqrt(sensor.P_diag_hist(2,:).*180/pi)*3, 'r--');
           subplot(4,1,3); grid on; hold on; title('Range');
           plot(sensor.t_hist, sensor.z_truth_hist(3,:), 'g-');
           plot(sensor.t_hist, sensor.z_est_hist(3,:), 'k-');
           plot(sensor.t_hist, sensor.z_est_hist(3,:)+sqrt(sensor.P_diag_hist(3,:))*3, 'r--');
           plot(sensor.t_hist, sensor.z_est_hist(3,:)-sqrt(sensor.P_diag_hist(3,:))*3, 'r--');
           subplot(4,1,4); grid on; hold on; title('Range Rate');
           plot(sensor.t_hist, sensor.z_truth_hist(4,:), 'g-');
           plot(sensor.t_hist, sensor.z_est_hist(4,:), 'k-');
           plot(sensor.t_hist, sensor.z_est_hist(4,:)+sqrt(sensor.P_diag_hist(4,:))*3, 'r--');
           plot(sensor.t_hist, sensor.z_est_hist(4,:)-sqrt(sensor.P_diag_hist(4,:))*3, 'r--');
           sensor.filter.plotTelemetry();
        end   
     
    end
    
end

