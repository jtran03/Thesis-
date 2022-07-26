function x = myStateTransitionFcn(x,u)

% Sample time (s)  
dk = 0.01; 

% Euler Discretiztion
x = x + [u(1)*cos(x(3))*dk;
         u(1)*sin(x(3))*dk;
         u(2)*dk];

end 