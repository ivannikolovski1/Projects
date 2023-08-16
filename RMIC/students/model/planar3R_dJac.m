%Forward kinematics
function dJac = planar3R_dJac(t,q,params)
global J_curr t_curr

J_prev = J_curr;
J_curr = planar3R_Jac(q,params);
t_prev = t_curr;
t_curr = t;
    

if t_curr == t_prev
dJac = zeros(3,3);
else 
    dJac = (J_curr-J_prev)/(t_curr-t_prev);
end;
end
