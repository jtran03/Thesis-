wheelBase = 0.4464; %m
wheelRadius = 0.1715/2; %m
timeStep = 0.1; % s or 10Hz  (setting a default time step) 

% Kalman Filter 
initialPose = [0, 0, 0]; 
initialErrorCovariance = [0.1 0 0; 0 0.1 0; 0 0 0.1]; 
Q = [0.1 0 0; 0 0.1 0; 0 0 0.1]; 
R = [0.5 0 0; 0 0.5 0; 0 0 0.5]; 



open_system('navigation.slx')