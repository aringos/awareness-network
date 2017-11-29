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
        costPerUnit     = 0;
        powerDrawPerUnit_W = 0.0;
        
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

        function sensor = Sensor(modelName, position, az)
            sensor.TB2I = [cos(az),-sin(az);sin(az),cos(az)];
            sensor.TI2B = sensor.TB2I';  
            sensor.pos  = position;
            minDetWidthPixels = 5;
            switch modelName
                case 'Delphi_Mid_ESR'
                    sensor.P_sizeBits = 32*3;
                    sensor.z_sizeBits = 32*3;
                    sensor.H    = [1;0;1;1];
                    sensor.max  = [90*pi/180; 0; 60; 9999];
                    sensor.R    = [0.5*pi/180 0 0 0; 
                                  0 0 0 0; 
                                  0 0 0.25 0;
                                  0 0 0 0.12];
                    sensor.dt   = 50e-3;
                    sensor.costPerUnit = 2500.0; 
                    sensor.powerDrawPerUnit_W = 18; 
                case 'Delphi_Long_ESR'
                    sensor.P_sizeBits = 32*3;
                    sensor.z_sizeBits = 32*3;
                    sensor.H    = [1;0;1;1];
                    sensor.max  = [20*pi/180; 0; 174; 9999];
                    sensor.R    = [0.5*pi/180 0 0 0; 
                                  0 0 0 0; 
                                  0 0 0.5 0;
                                  0 0 0 0.12];
                    sensor.dt   = 50e-3;
                    sensor.costPerUnit = 2500.0; 
                    sensor.powerDrawPerUnit_W = 18;
                case 'Velodyne_VLP16'
                    sensor.P_sizeBits = 32*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H    = [1;0;1;0];
                    sensor.max  = [180*pi/180; 0; 100; 0];
                    sensor.R    = [0.05 0 0 0; ...
                                   0 0 0 0; ...
                                   0 0 0.03 0; ...
                                   0 0 0 0];
                    sensor.costPerUnit = 8000.0; 
                    sensor.powerDrawPerUnit_W = 8;
                case 'Arducam_OV7670'
                    sensor.P_sizeBits = 32*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H    = [1;1;0;0];
                    FOV         = 25.0*pi/180;
                    iFOV        = FOV/640.0;
                    sensor.R    = [iFOV  0    0 0; ...
                                  0      iFOV 0 0; ...
                                  0      0    0 0; ... 
                                  0      0    0 0];
                    sensor.dt   = 1.0/30.0;
                    pxRange     = 1.2/(2*minDetWidthPixels*tan(1.5*iFOV));
                    sensor.max  = [FOV; 1e9; pxRange; 0];
                    sensor.costPerUnit = 14.0;
                    sensor.powerDrawPerUnit_W = 1.3;
                case 'Arducam_OV5647'
                    sensor.P_sizeBits = 32*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H    = [1;1;0;0];
                    FOV         = 54.0*pi/180;
                    iFOV        = FOV/1920.0;
                    sensor.R    = [iFOV  0    0 0; ...
                                  0      iFOV 0 0; ...
                                  0      0    0 0; ... 
                                  0      0    0 0];
                    sensor.dt   = 1.0/30.0;
                    pxRange     = 1.2/(2*minDetWidthPixels*tan(1.5*iFOV));
                    sensor.max  = [FOV; 1e9; pxRange; 0];
                    sensor.costPerUnit = 15.0;      
                    sensor.powerDrawPerUnit_W = 1.3;
                case 'Raspberry_Pi_Camera_1080p30'
                    sensor.P_sizeBits = 32*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H    = [1;1;0;0];
                    FOV         = 62.0*pi/180;
                    iFOV        = FOV/1920.0;
                    sensor.R    = [iFOV  0    0 0; ...
                                  0      iFOV 0 0; ...
                                  0      0    0 0; ... 
                                  0      0    0 0];
                    sensor.dt   = 1.0/30.0;
                    pxRange     = 1.2/(2*minDetWidthPixels*tan(1.5*iFOV));
                    sensor.max  = [FOV; 1e9; pxRange; 0];
                    sensor.costPerUnit = 25.0;
                    sensor.powerDrawPerUnit_W = 1.3;
                case 'Raspberry_Pi_Camera_720p60'
                    sensor.P_sizeBits = 32*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H    = [1;1;0;0];
                    FOV         = 62.0*pi/180;
                    iFOV        = FOV/1280.0;
                    sensor.R    = [iFOV  0      0 0; ...
                                  0      iFOV/2 0 0; ...
                                  0      0      0 0; ... 
                                  0      0      0 0];
                    sensor.dt   = 1.0/30.0; %Actual 1/60, SP limited (better thetadot though)
                    pxRange     = 1.2/(2*minDetWidthPixels*tan(1.5*iFOV));
                    sensor.max  = [FOV; 1e9; pxRange; 0];
                    sensor.costPerUnit = 25.0;
                    sensor.powerDrawPerUnit_W = 1.3;
                case 'Pi_12mmM12'
                    sensor.P_sizeBits = 32*2;
                    sensor.z_sizeBits = 32*2;
                    sensor.H    = [1;1;0;0];
                    FOV         = 28.0*pi/180;
                    iFOV        = FOV/1920.0;
                    sensor.R    = [iFOV  0    0 0; ...
                                  0      iFOV 0 0; ...
                                  0      0    0 0; ... 
                                  0      0    0 0];
                    sensor.dt   = 1.0/30.0;
                    pxRange     = 1.2/(2*minDetWidthPixels*tan(1.5*iFOV));
                    sensor.max  = [FOV; 1e9; pxRange; 0];
                    sensor.costPerUnit = 25.0+10.0+10.0;  
                    sensor.powerDrawPerUnit_W = 1.3;
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
            sensor.color    = rand(3,1).*.7;
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

