function [ obs ] = triangulateObservations(anglePacket1, anglePacket2)

    tAlign = mean([anglePacket1.observation_t; anglePacket2.observation_t]);
    z1     = timeAlign(anglePacket1, tAlign);
    z2     = timeAlign(anglePacket2, tAlign);
    
    [u1, u1dot] = getLOS(z1);
    [u2, u2dot] = getLOS(z2);
    
    vecS1toS2 = anglePacket2.sensor_pos - anglePacket1.sensor_pos;
    theta1    = acos(dot(u1,vecS1toS2)/norm(vecS1toS2));
    theta2    = acos(dot(u2,-vecS1toS2)/norm(-vecS1toS2));
    
    [z1(3), z1(4)] = triangulate(u1, u1dot, u2, u2dot, vecS1toS2);
    [z2(3), z2(4)] = triangulate(u2, u2dot, u1, u1dot, -vecS1toS2);
    
    P1 = triVariance(z1, z2, theta1, theta2, anglePacket1.observation_P, anglePacket2.observation_P);
    P2 = triVariance(z2, z1, theta1, theta2, anglePacket2.observation_P, anglePacket1.observation_P);
    
    obs = [FusionObservation(z1, P1, tAlign, anglePacket1.sensor_pos); ...
           FusionObservation(z2, P2, tAlign, anglePacket2.sensor_pos)];
    obs(1).type = 'TRIANGULATED';
    obs(2).type = 'TRIANGULATED';
end

function P = triVariance(z1, z2, theta1, theta2, Rangle1, Rangle2)
    psi      = pi-theta1-theta2;
    R_var    = z1(3)*z1(3) * cos(psi)*cos(psi) * Rangle1(1,1);
    R_var    = R_var + z2(3)*z2(3)*Rangle2(1,1);
    R_var    = R_var / (sin(psi)*sin(psi));
    Rdot_var = (z1(3)*z1(3)+z2(3)*z2(3))*(Rangle1(2,2)*Rangle1(2,2));
    Rdot_var = Rdot_var / (sin(psi)*sin(psi));
    P        = [Rangle1(1,1) 0            0     0; ...
                0            Rangle1(2,2) 0     0; ...
                0            0            R_var 0; ...
                0            0            0     Rdot_var];
                
end

function [R, Rdot] = triangulate(u1, u1dot, u2, u2dot, D)
    R      = (D(1)*u2(2)-D(2)*u2(1))/(u1(1)*u2(2)-u1(2)*u2(1));
    f      = D(1)*u2(2)-D(2)*u2(1);
    g      = u1(1)*u2(2)-u1(2)*u2(1);
    fPrime = D(1)*u2dot(2)-D(2)*u2dot(1);
    gPrime = u1dot(1)*u2(2)+u1(1)*u2dot(2)-u1dot(2)*u2(1)-u1(2)*u2dot(1);
    Rdot   = (fPrime*g-f*gPrime) / (g*g);
end

function [u, udot] = getLOS(z)
    u    = [cos(z(1));  sin(z(1)) ];
    udot = [-z(2)*u(2); z(2)*u(1) ];   
end

function z = timeAlign(anglePacket, tAlign)
    z    = anglePacket.observation_z;
    dt   = tAlign - anglePacket.observation_t;
    z(1) = z(1) + z(2)*dt;
end

