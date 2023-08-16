% *************************************************************************
%                2-Link Point Mass Manipulator Parameters                 *
% *************************************************************************

% Assign values to variables

p1  = 3.473;
p2  = 0.196;
p3  = 0.242;

% Joint friction coefficients
fd1 = 5.3;
fd2 = 1.1;

K = 100*eye(2);
alpha = 10*eye(2);
gamma = 10*eye(5);
beta = 1;