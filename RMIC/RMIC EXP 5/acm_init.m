% *************************************************************************
%                2-Link Point Mass Manipulator Parameters                 *
% *************************************************************************

% Assign values to variables

p1  = 3.473;
p2  = 0.196;
p3  = 0.242;

% Joint friction coefficients
fd1 = 5.3;
fd2 = 1.1;
K = 10*eye(2);
alpha = 10*eye(2);
gamma = 2*eye(5);
beta = 1;


%%
%     syms q1(t) q2(t) dq1(t) dq2(t) t alpha tau
% %     q1  = x(1);
% %     dq1 = x(2);
% %     q2  = x(3);
% %     dq2 = x(4);
%     x  = [q1; dq1; q2; dq2];
%     
%     M      = zeros(2,2);
%     M      = sym('m',[2 2]);
%     M(1,1) = p1 + 2*p3*cos(q2);
%     M(1,2) = p2 + p3*cos(q2);
%     M(2,1) = p2 + p3*cos(q2);
%     M(2,2) = p2;
% 
%     C      = zeros(2,2);
%     C      = sym('c',[2 2]);
%     C(1,1) = -p3*sin(q2)*dq2;
%     C(1,2) = -p3*sin(q2)*(dq1+dq2);
%     C(2,1) = p3*sin(q2)*dq1;
%     C(2,2) = 0;
% 
%     Fd     = diag([fd1 fd2]);
%     
%     % The system without bounded disturbances =============================
%     ddq = M\(tau - C*[dq1;dq2] - Fd*[dq1;dq2]);
%     dx  = [dq1; ddq(1); dq2; ddq(2)];
%     
%      qd  = [sin(3*t);2*cos(t)];
%      dqd = [diff(qd(1),t);diff(qd(2),t)];
%      ddqd= [diff(dqd(1),t);diff(dqd(2),t)];
%     e   = [q1;q2]-qd
%     de  = jacobian(e,t)
%     de  = subs(de,[diff(q1(t), t) diff(q2(t), t)],[dq1 dq2])
%     dde = jacobian(de,t);
%     r   = e-alpha*de;
%     dr  = de-alpha*dde;
%     wanted = M*ddqd+alpha*M*de+C*(dqd+alpha*e);
%     wanted = simplify(wanted) 