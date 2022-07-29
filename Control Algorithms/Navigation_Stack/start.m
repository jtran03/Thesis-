wheelBase = 0.4464; %m
wheelRadius = 0.1715/2; %m

% Kalman Filter 
initialPose = [0, 0, 0]; %[x, y, theta]
initialInput = [0, 0]; % [v, w] 
timeStep = 0.1; % Kalman Sample Rate in s 
initialErrorCovariance = [0.1 0 0; 0 0.1 0; 0 0 0.1]; 

Q = [0.1 0 0; 0 0.1 0; 0 0 0.1]; 
R = [0.5 0 0; 0 0.5 0; 0 0 0.5]; 



open_system('navigation.slx')