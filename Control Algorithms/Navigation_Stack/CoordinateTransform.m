function y = CoordinateTransform(x, y, theta, pose)
    
    pose2D = [pose(1,1), pose(2,1), 1]; 
    
    heading = theta - pose(3,1);
    
    transformMatrix = [cos(heading), -sin(heading), x;
                       sin(heading), cos(heading), y; 
                       0,          0,          1]; 
                   
    poseTransform = pose2D*transformMatrix; 
    resultX = poseTransform(1,1);
    resultY = poseTransform(2,1); 
    
    y = [resultX, resultY, heading]; 
end 