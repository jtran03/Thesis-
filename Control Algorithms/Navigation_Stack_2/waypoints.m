%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% User Input % x, y, curve radius
mainWaypoints = [0, 10, 0;
                -1, 5, 1;
                -5, 5, 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Vectors 
vectorA = mainWaypoints(1,[1,2]) - mainWaypoints(2,[1,2]); 
vectorB = mainWaypoints(3,[1,2]) - mainWaypoints(2,[1,2]); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determine Cross Product and Dot Product
dotAB = vectorA(1,1)*vectorB(1,1) + vectorA(1,2)*vectorB(1,2); 
crossAB = vectorA(1,1)*vectorB(1,2) - vectorA(1,2)*vectorB(1,1); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determine Angle 
alpha = atan2(crossAB, dotAB); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CIRCLE RADIUS
smallRadius = mainWaypoints(2,3); 
bigRadius = smallRadius/sin(alpha/2); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determine some parameters 
unitVectorA = abs(bigRadius)*(vectorA/norm(vectorA)); 
rotMatrix = [cos(alpha/2) -sin(alpha/2); sin(alpha/2)  cos(alpha/2)]; 
bigCircVector = rotMatrix*unitVectorA';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CIRCLE CENTRE
bigCircleCentre = mainWaypoints(2,[1,2]); 
smallCircleCentre = bigCircleCentre + bigCircVector';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LINE GRADIENTS
gradientA = vectorA(1,2)/vectorA(1,1);
gradientB = vectorB(1,2)/vectorB(1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LINE INTERCEPTS y = x + b --> b = y - mx
intercptA = mainWaypoints(2,2) - gradientA*mainWaypoints(2,1);
intercptB = mainWaypoints(2,2) - gradientB*mainWaypoints(2,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INVERSE LINES GRADIENTS
invGradientA = -1/gradientA; 
invGradientB = -1/gradientB; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INVERSE LINE INTERCEPTS 
invIntercptA = smallCircleCentre(1,2) - invGradientA*smallCircleCentre(1,1); 
invIntercptB = smallCircleCentre(1,2) - invGradientB*smallCircleCentre(1,1); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% POI LINE A
xPOIA = (invIntercptA - intercptA)/(gradientA - invGradientA); 
yPOIA = invIntercptA*xPOIA + invIntercptA;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot 
hold on 
plot(mainWaypoints([1, 2], 1), mainWaypoints([1, 2], 2)); 
plot(mainWaypoints([2, 3], 1), mainWaypoints([2, 3], 2)); 
th = 0:pi/50:2*pi;
BigCircleX = bigRadius * cos(th) + bigCircleCentre(1,1);
BigCircleY = bigRadius * sin(th) + bigCircleCentre(1,2);
plot(BigCircleX, BigCircleY);
smallCircleX = smallRadius * cos(th) + smallCircleCentre(1,1);
smallCircleY = smallRadius * sin(th) + smallCircleCentre(1,2);
plot(smallCircleX, smallCircleY);
plot(mainWaypoints(2,1), mainWaypoints(2,2), 'o'); 
plot(unitVectorA(1,1), unitVectorA(1,2));
plot(xPOIA, yPOIA, 'o'); 
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


startWaypoint = [0 0]; 
endWaypoint = [10 0];
percentage = 0.7; 
smallRadius = 0.5; 
NoLineSegments = 11;

% Generate y = mx + b 
m = (endWaypoint(2) - startWaypoint(2))/(endWaypoint(1) - startWaypoint(1)); 
b = startWaypoint(2) - 3*startWaypoint(1);  

% Check if b is NaN 
if isnan(b)
    b = 0; 
end

% Generate Ax + By + C = 0 
A = m; 
B = -1; 
C = b; 

% Generate a circle at 50% of a line
centreCircle = startWaypoint + percentage*(endWaypoint - startWaypoint); 

% Determine perpendicular distance 



% Check if line is vertical = linspace in the y direction 
if isinf(m)
    ycoords = transpose(linspace(startWaypoint(2),endWaypoint(2),NoLineSegments+1));
    xcoords = ycoords*0; 
else
    xcoords = transpose(linspace(startWaypoint(1),endWaypoint(1),NoLineSegments+1));
    ycoords = (m*xcoords + b); 
end
