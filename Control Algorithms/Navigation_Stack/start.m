% System Parameters 
LIVE_MODE = 1; % turn on live mapping 
wheelBase = 0.4464; %m
wheelRadius = 0.1715/2; %m
initialPose = [0, 0, 0]; %[x, y, theta]
initialInput = [0; 0]; % [v, w] 
timeStep = 0.1; % Kalman Sample Rate in s 

%Waypoint Navigation
firstGoal = [1.5,1.5]; 
secondGoal = [0.5,0.5]; 

%Sample Rates
encoderSampleRate = 10; %Hz 


% Transformation matrices
% World to RobotOdom
W2ORx = 0.5; 
W2ORy = 0.5; 
W2ORtheta = pi/2;  
W2ORTransform = [cos(W2ORtheta), -sin(W2ORtheta), W2ORx; 
                sin(W2ORtheta), cos(W2ORtheta), W2ORy; 
                0            , 0            , 1]; 

% World to LidarOdom 
W2LOx = 0.5 - 0.0073926; 
W2LOy = 0.5 + 0.32076;
W2LOtheta = (3/2)*pi; 
W2LOTransform = [cos(W2LOtheta), -sin(W2LOtheta), W2LOx; 
                sin(W2LOtheta), cos(W2LOtheta), W2LOy; 
                0            , 0            , 1]; 

% Lidar to Robot Values 
L2Rx = 0.32076;  % x
L2Ry = -0.0073926; % y
L2Rtheta = pi;   % theta (rad) 
L2RTransform = [cos(L2Rtheta), -sin(L2Rtheta), L2Rx; 
                sin(L2Rtheta), cos(L2Rtheta), L2Ry; 
                0            , 0            , 1]; 

% Kalman Filter parameters 
xNum = 3; % Number of states
yNum = 3; % Number of outputs
uNum = 2; % Number of inputs
QVar = 0.1; % R variable number
RVar = 0.5; % Q variance number 
PInit = 0.1; % Initial error covariance matrix

% Generate relevant matrices =======================================
initialErrorCovariance = diag(PInit*ones(xNum,1)); % Initial P{k-1} 
Q = diag(QVar*ones(xNum,1)); % Q matrix
R = diag(RVar*ones(yNum,1)); % R matrix 
C = [1 0 0; 
     0 1 0; 
     0 0 1];
D = [0 0; 
     0 0; 
     0 0];
         
open_system('navigation.slx')