% Information matrix
% * NOTE: students should build the whole function having 'planar3R_Y'
function Gamma_b = calc_Gamma(Q, dQ, ddQ, params)
    % This function should calculate the information matrix Gamma_b.
    % Hint: Use the function "planar3R_Y" to calculate the minimal parameter
    % regressor of coriolis joint torque vector 
    %
    % <<YOUR CODE HERE>>
    Gamma_b = zeros(3*size(Q,2),9);
    for i=1:size(Q,2)
        Y_reg=planar3R_Y(Q(1:3,i), dQ(1:3,i), ddQ(1:3,i), params.g_base, params.pkin(1:2));
        Gamma_b(3*i-2:3*i,:)=Y_reg;
    end
end

