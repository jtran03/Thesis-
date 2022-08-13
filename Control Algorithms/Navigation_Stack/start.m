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
% W2LOx = 0.5 - 0.0073926; 
% W2LOy = 0.5 + 0.32076;
% W2LOtheta = (3/2)*pi; 
% W2LOTransform = [cos(W2LOtheta), -sin(W2LOtheta), W2LOx; 
%                 sin(W2LOtheta), cos(W2LOtheta), W2LOy; 
%                 0            , 0            , 1]; 

% World to LidarOdom
W2LOTransform = W2ORTransform/(L2RTransform); 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kalman Filters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
% Kalman Parameters
xNum = 3; % Number of states
yNum = 3; % Number of outputs
uNum = 2; % Number of inputs
QVar = 0.1; % R variable number
RVar = 0.5; % Q variance number 
PInit = 0.1; % Initial error covariance matrix

% Generate relevant matrices 
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