waypoints = [1,1];
startPose = [0,0];
endPose = waypoints(1,:); 
heading = atan2((endPose(1,2) - startPose(1,2)),(endPose(1,1) - startPose(1,1))); 
