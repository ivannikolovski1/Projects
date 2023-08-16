function [q_d, dq_d, ddq_d, x_d, dx_d, ddx_d] = motion_generator(t, params)
    q_d   = zeros(3,1);
    dq_d  = zeros(3,1);
    ddq_d = zeros(3,1);
    x_d   = zeros(3,1);
    dx_d  = zeros(3,1);
    ddx_d = zeros(3,1);

    switch params.controller
        case 0
            % Excercise I
            % Devise a suitable motion for the robot in order to identify its
            % base parameters
            % <<YOUR CODE HERE>>
            % Use the following initial joint position as a tested basis, just
            % add it to a relative motion
            q0 = params.q0;
            %
            q_d=0.2*sin(q0*t)+0.1*cos(q0/3*t)-0.3*cos(q0/5*t);
            dq_d=0.2*q0.*cos(q0*t)-0.1/3*q0.*sin(q0/3*t)+0.3/5*q0.*sin(q0/5*t);
            ddq_d=-0.2*(q0.^2).*sin(q0*t)-0.1/9*(q0.^2).*cos(q0/3*t)+0.3/25*q0.^2.*sin(q0/5*t);
        case 1
            % Exercise III
            % Implement a simple trajectory that the impedance controller is 
            % supposed to follow (e.g. a sinusoidal motion)
            % <<YOUR CODE HERE>>
            q0 = params.q0;
            J = planar3R_Jac(q0,params);
            x0 = J*q0;
            x_d(1:2)=0.3*sin(x0(1:2)*4*t);
            dx_d(1:2)=-x0(1:2)*0.3*4.*cos(x0(1:2)*4*t);
            ddx_d(1:2)=-x0(1:2).^2*0.3*16.*sin(x0(1:2)*4*t);
%             x_d=[5;2;1];
%             dx_d=[1;1;1];
%             ddx_d=[0;0;0];
        case 2
            % Exercise IV
            % <<YOUR CODE HERE>>
            q0 = params.q0;
            J = planar3R_Jac(q0,params);
            x0 = J*q0;
            x_d=0.2*sin(x0*t);
%             dx_d=-x0*0.2.*cos(x0*t);
%             ddx_d=-x0.^2*0.2.*sin(x0*t);
        otherwise
    end
end

