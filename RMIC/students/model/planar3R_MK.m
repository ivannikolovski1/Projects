function MK = planar3R_MK(q,params)
    % --> Already here
    J = planar3R_Jac(q,params);
    M = planar3R_M_reg(q,params);

    % <<YOUR CODE HERE>>
    MK = eye(3);
    MK    = inv(J')*M*inv(J);
end 
