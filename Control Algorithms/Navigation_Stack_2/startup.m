%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Parameters (Change at run time) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
LIVE_MODE = 0; % turn on live mapping 

setInitialPose = [0.5, 0.5, pi/2]; %[x, y, theta] w.r.t World 
initialInput = [0; 0]; % [v, w] 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Important Waypoints  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
chargingStn = [0; 0]; 
loadingStn = [1.5; 1.5]; 
unloadingStn = [1.0; 1.0]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot Parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
wheelBase = 0.4464; %m
wheelRadius = 0.1715/2; %m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transformation matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% World to RobotOdom
W2ORx = setInitialPose(1,1); 
W2ORy = setInitialPose(1,2); 
W2ORtheta = setInitialPose(1,3);  
W2ORTransform = [cos(W2ORtheta), -sin(W2ORtheta), W2ORx; 
                sin(W2ORtheta), cos(W2ORtheta), W2ORy; 
                0            , 0            , 1]; 

% Lidar to Robot 
L2Rx = 0.32076;  % x
L2Ry = -0.0073926; % y
L2Rtheta = pi;   % theta (rad) 
L2RTransform = [cos(L2Rtheta), -sin(L2Rtheta), L2Rx; 
                sin(L2Rtheta), cos(L2Rtheta), L2Ry; 
                0            , 0            , 1]; 

% World to LidarOdom
W2LOTransform = W2ORTransform/(L2RTransform); 

open_system('navigation.slx')