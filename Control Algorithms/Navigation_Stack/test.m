L2RTransform = [0.0073926; -0.32076; (3/2)*pi];

pose = [0;0;0];

x = L2RTransform(1,1);
y = L2RTransform(2,1);
theta = L2RTransform(3,1); 

pose2D = [pose(1,1), pose(2,1), 1]; 

heading = theta - pose(3,1);

transformMatrix = [cos(heading), -sin(heading), x;
                   sin(heading), cos(heading), y; 
                   0,          0,          1]; 

poseTransform = pose2D*transformMatrix; 

resultX = poseTransform(1,1);
resultY = poseTransform(1,2); 

y = [resultX, resultY, heading]; 