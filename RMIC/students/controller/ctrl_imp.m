% Impedance controller that calculates the desired torque based on 
% Cartesian motion error and given impedance parameters
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
function tau = ctrl_imp(t, q, dq, x_d, dx_d, ddx_d, params)
    % Stiffness and damping factors
    K_x  = diag([150;100;50]);
    x    = planar3R_fkin(q,params);
    J    = planar3R_Jac(q,params);
    MK   = planar3R_MK(q,params);
    CK   = planar3R_CK(t,q,dq,params);
    taug = planar3R_g_reg(q,params);
    D_x  = damping_design(K_x, MK);

    % <<YOUR CODE HERE>>
    dx       = J*dq;
    delta_x  = x_d-x;
    delta_dx = dx_d-dx;
    tau      = zeros(3,1);
    tau      = J'*(MK*ddx_d+CK*dx_d+D_x*delta_dx+K_x*delta_x)+taug;
end

