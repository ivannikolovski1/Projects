% ************************************************************************* 
% PD controller that calculates the desired torque based on a given
% motion error
% *************************************************************************

% Input:
% t [s]
%   Current time
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% q_d [3x1]
%   Desired generalized coordinates (joint angles) [rad]
% dq [3x1]
%   Generalized velocities (joint velocities) [rad/s]
% 
% Output:
% tau [3x1]
%   Desired joint torques [Nm]
%
% 
% (C) Lehrstuhl für Robotik und Systemintelligenz, TUM
function tau = ctrl_PD(t, q, q_d, dq, dq_d)
    %% Exercise I
    % From the given desired and current joint angles the pd controller should
    % calculate a torque
    % <<YOUR CODE HERE>>
    tau = zeros(3,1);
    P= 50;
    D= 0.03;
    tau = P*(q_d-q)+D*(dq_d-dq);
end

