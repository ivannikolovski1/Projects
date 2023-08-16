function MK = planar3R_MK_R(q,params)
    % Here you have the Jacobian and mass matrix in joint space
    J = planar3R_Jac_R(q,params);
    M = planar3R_M_reg(q,params);

    % <<YOUR CODE HERE>>
    MK    = eye(2);
    MK    = pinv(J')*M*pinv(J);
    
end 
