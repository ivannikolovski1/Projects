function CK = planar3R_CK_R(t,q,dq,params)
    % Here you have the Jacobian and Coriolis matrix in joint space
    J  = planar3R_Jac_R(q,params);
    % dJ = planar3R_dJac_R(t,q,params);
    dJ = planar3R_dJac_R_analytic(t, q, dq, params);
    C  = planar3R_C_reg(q, dq, params);
    
    % Here is the mass matrix in task (Cartesian) space
    MK = planar3R_MK_R(q,params);
    
    % <<YOUR CODE HERE>>
    % Write the expression for the Coriolis matrix in task space
    CK = eye(2);
    CK = (pinv(J')*C-MK*dJ)*pinv(J);
end 
