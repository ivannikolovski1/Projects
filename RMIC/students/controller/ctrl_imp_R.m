% Impedance controller that calculates the desired torque based on 
% Cartesian motion error (translation only) and given impedance parameters
% 
% Input:
% t [s]
%   Current time
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% dq [3x1]
%   Generalized velocities (joint velocities) [rad/s]
% x_d [3x1]
%   Desired Cartesian position [m]
% dx_d [3x1]
%   Desired Cartesian velocities [m/s]
% ddx_d [3x1]
%   Desired Cartesian accelerations [m/s^2]
% params
%   Robot parameters
% 
% Output:
% tau [3x1]
%   Desired joint torques [Nm]
%
% 
% (C) Lehrstuhl für Robotik und Systemintelligenz, TUM
function tau = ctrl_imp_R(t, q, dq, x_d, dx_d, ddx_d, params)
    K_x  = diag([500;500]);
    x    = planar3R_fkin_R(q, params);
    J    = planar3R_Jac_R(q, params);
    MK   = planar3R_MK_R(q, params);
    CK   = planar3R_CK_R(t, q, dq, params);
    taug = planar3R_g_reg(q, params);
    D_x  = damping_design(K_x, MK);

    % <<YOUR CODE HERE>>
    dx       = J*dq;
    delta_x  = x_d(1:2)-x;
    delta_dx = dx_d(1:2)-dx;
    tau      = zeros(3,1);
    tau      = J'*(MK*ddx_d(1:2)+CK*dx_d(1:2)+D_x*delta_dx+K_x*delta_x)+taug;
end

