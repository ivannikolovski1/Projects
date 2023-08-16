% Calculates dLambda which is integrated and used by the collision observer
% 
% Input:
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% dq [3x1]
%   Generalized velocities (joint velocities) [rad/s]
% tau [3x1]
%   Desired torques of the current time step [Nm]
% r [3x1]
%   residual from the momentum observer [Nm]
% params
%   Robot parameters
% 
% Output:
% dp_hat
%   Derivative of the momentum estimate p_hat which is used by the 
%   collision observer
% 
% (C) Lehrstuhl für Robotik und Systemintelligenz, TUM
function dp_hat = calc_dp_hat(q, dq, tau, r, params)
    taug_reg = planar3R_g_reg(q, params);     % gravity torque
    C_reg    = planar3R_C_reg(q, dq, params); % Coriolis matrix
    
    % <<YOUR CODE HERE>>    
    dp_hat   = zeros(3,1);
    beta_hat = taug_reg-C_reg'*dq;
    dp_hat   = tau-beta_hat+r;
end

