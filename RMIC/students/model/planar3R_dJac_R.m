%Forward kinematics
function dJac = planar3R_dJac_R(t,q,params)
    global J_curr_R t_curr

    J_prev = J_curr_R;
    J_curr_R = planar3R_Jac_R(q,params);
    t_prev = t_curr;
    t_curr = t;


    if t_curr == t_prev
    dJac = zeros(2,3);
    else 
        dJac = (J_curr_R-J_prev)/(t_curr-t_prev);
    end
end
