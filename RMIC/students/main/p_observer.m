function r = p_observer(q, dq, p_hat, params)
    if params.flag_obs == true
        M_reg = planar3R_M_reg(q, params);
        % <<YOUR CODE HERE>>
        r = zeros(3,1);
        p = M_reg*dq;
        r = params.K_O*(p-p_hat);
    else
        r = zeros(3,1);
    end
end

