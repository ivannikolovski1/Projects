%Forward kinematics
function xEE = planar3R_fkin_R(q,params)
 
l1 = params.pkin(1);
l2 = params.pkin(2);
l3 = params.pkin(3);
q1 = q(1);
q2 = q(2);
q3 = q(3);
    
%Cartesian positions of the joints of ROBOT at each time step  
xEE = [l1*cos(pi/2+q1)+l2*cos(pi/2+q1+q2)+l3*cos(pi/2+q1+q2+q3);l1*sin(pi/2+q1)+l2*sin(pi/2+q1+q2)+l3*sin(pi/2+q1+q2+q3)];
    
end
