%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Parameters (Change at run time) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
LIVE_MODE = 1; % turn on live mapping 

setInitialPose = [0, 0, pi/2]; %[x, y, theta] w.r.t World 
initialInput = [0; 0]; % [v, w] 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Important Waypoints  rros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
chargingStn = [setInitialPose(1,1), setInitialPose(1,2)]; 
loadingStn = [0, 10]; 
unloadingStn = [3, 10]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot Parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
wheelBase = 0.4464; %m
wheelRadius = 0.1715/2; %m
robotLength = 0.775; %m 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load Map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simMapImage = imread('maps/sim_map.pgm'); 
[simRows, simColoumns, ~] = size(simMapImage);
simMapCrop = simMapImage(simColoumns/2 - 525:simColoumns/2, simRows/2:simRows - 1551); 
simMapPad = padarray(simMapCrop, [1 1], 0, 'both');
logicalMap = simMapPad < 100; 

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open Simulink
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% map = binaryOccupancyMap(logicalMap, 1/0.05);
% inflate(map, 0.775/2);
% prm = mobileRobotPRM(map, 100);
% show(map);
open_system('nav2.slx');