function CK = planar3R_CK(t,q,dq,params)
    % --> Already here
    J  = planar3R_Jac(q,params);
    C  = planar3R_C_reg(q, dq, params);
    MK = planar3R_MK(q,params);
    %dJ = planar3R_dJac(t,q,params);
    dJ = planar3R_dJac_analytic(t, q, dq, params);

    % <<YOUR CODE HERE>>
    CK = eye(3);
    CK = (inv(J')*C-MK*dJ)*inv(J);
end 
