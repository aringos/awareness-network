function [ vectorOfFovVertices ] = drawSensorFOVs( sensors )
%Return a list of vertices to draw the FOV of each sensor

verticesPerSide = 100;

vectorOfFovVertices = {};

for sensorIdx=1:length(sensors)
    % Build the vertex vector in sensor LOS coordinates 
    % and then rotate to reference frame
    origin  = sensors(sensorIdx).pos;
    range   = sensors(sensorIdx).max(3); 
    fov     = sensors(sensorIdx).max(1);
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
    leftRadialLine = fliplr(leftRadialLine');
    rightRadialLine = fliplr([leftRadialLine(1,:); -leftRadialLine(2,:)]);
    thisVertexVector = [leftFovLine leftRadialLine rightRadialLine rightFovLine];
    %Transform to inertial
    thisVertexVector = origin+sensors(sensorIdx).TB2I*thisVertexVector;
    
    vectorOfFovVertices
end


end

