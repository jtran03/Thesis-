dubConnObj = dubinsConnection('DisabledPathTypes', {'LSL'}); 
dubConnObj.MinTurningRadius = 0.5;

startPose = [0 0 0]; 
goalPose = [0 1 pi]; 

[pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);

show(pathSegObj{1})

length = pathSegObj{1}.Length;
poses = interpolate(pathSegObj{1},0:0.2:length);

disp(poses);