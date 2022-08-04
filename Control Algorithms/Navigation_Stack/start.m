% System Parameters 
LIVE_MODE = 1; % turn on live mapping 
wheelBase = 0.4464; %m
wheelRadius = 0.1715/2; %m
initialPose = [0, 0, 0]; %[x, y, theta]
initialInput = [0; 0]; % [v, w] 
timeStep = 0.1; % Kalman Sample Rate in s 

%Waypoint Navigation
startLoc = [0,0]; 
goalLoc = [1,1]; 

%Sample Rates
encoderSampleRate = 10; %Hz 


% Transformation matrices
% Lidar to Robot Values 
L2Rx = 0.0073926; % x
L2Ry = -0.32076;  % y
L2Rtheta = (3/2)*pi;   % theta (rad) 

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
 
L2RTransform = [L2Rx; L2Ry; L2Rtheta];
 
open_system('navigation.slx')