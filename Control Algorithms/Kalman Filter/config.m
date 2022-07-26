% Constants
radius = 0.1715; 
PPR = 4096.0; 
driveLength = 0.446;
sampleFreq = 1; %Hz
dk = 1/sampleFreq; %s 
diffDriveConstant = [radius; PPR; driveLength;];

%Initial Pose: x, y, theta 
initialPose = [0.0; 0.0; 0.0];

currentPose = [0.0; 0.0; 0.0]; 



% Encoder Values 
prevLeftEncoder = 0.0; 
prevRightEncoder = 0.0; 


% Process noise covariance
Q = 1e-3; 

%Measurement noise covariance
R = 5e-3;

%Sampling Time
dk = 0.01; %{s}



open_system('EKF.slx');