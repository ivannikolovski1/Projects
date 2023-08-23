function minimal_example
    addpath('./mynmpcroutine');
    mpciterations = 70;
    N             = 10;
    T             = 0.2;
    tmeasure      = 0.0;
    iprint        = 10;
    xmeasure      = [40 -4 0 30];
    u0            = zeros(2,N);
    type          = 'difference equation';
    tol_opt       = 1e-8;
    opt_option    = 0;
    atol_ode_real = 1e-12;
    rtol_ode_real = 1e-12;
    atol_ode_sim  = 1e-4;
    rtol_ode_sim  = 1e-4;
    Q=diag([0,0.25,0.2,10]);
    R=diag([0.33,5]);
    zinital=[100 25 -4 0;
        -40 25 0 0];
        

    

   [t  ,x  ,u  , z ]=mynmpc(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system, ...
         mpciterations, N, T, tmeasure, xmeasure, u0,  ...
         tol_opt, opt_option, ...
         type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
         iprint, @printHeader, @printClosedloopData, @plotTrajectories,Q,R,zinital);
     

  
%    [xcontinuos]=newIntegratedState(x,u,T);
    

   
  
    
     
    rmpath('./mynmpcroutine');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = runningcosts(t, x, u, xref, Q, R) %LANE DETECTION
cost=(x-xref)*Q*(x-xref)'+u'*R*u;
end

function cost = terminalcosts(t, x)
    cost = 0.0;
end

function [c,ceq] = constraints(t, x, u, qx, qy , qt, m)
    minv = 0 ;
    maxv = 40;
    roadwidth=12;
    c(1) = x(2)-roadwidth/2;
    c(2) = -x(2)-roadwidth/2;
    c(3) = x(4)-maxv;
    c(4) = -x(4)-minv;
%     c(5) = x(3)-0.6;
%     c(6) = -x(3)-0.2;
%     c(7) = qx*x(1)+qy*x(2)+qt;

%      c=[c;qx'*x(1)+qy'*x(2)+qt'];

    
    for k=1:m
    c(4+k) = qx(:,k)*x(1)+qy(:,k)*x(2)+qt(:,k);
    end
  %for more than one vehicle we have a vector inequality do i need a for loop to check for all entries
    ceq = [];
end

function [c,ceq] = terminalconstraints(t, x, qx ,qy , qt, m)
    minv = 0 ;
    maxv = 40;
    roadwidth=12;
    c(1) = x(2)-roadwidth/2;
    c(2) = -x(2)-roadwidth/2;
    c(3) = x(4)-maxv;
    c(4) = -x(4)-minv;
%     c(5) = x(3)-0.6;
%     c(6) = -x(3)-0.2;
%     c(7) = qx*x(1)+qy*x(2)+qt;
    for k=1:m
    c(4+k) = qx(:,k)*x(1)+qy(:,k)*x(2)+qt(:,k);
    end
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [-9,-0.4];
    ub  = [5,0.4];
end

function y = system(t, x, u, T, xmeasure)

    lr=2;
    lf=2;
    v=x(4);
    phi=x(3);
    % a=u(1);
    delta=u(2);
    [ds,dd,dphi,dv]=kinematicbicycle(v,phi,0,0);
    % alpha=atan((lr/(lr+lf)*tan(delta));
    z1=phi+atan(lr*tan(delta)/(lr+lf));
    z2=T^2*v*tan(delta);
    z3=((lr*tan(delta))^2/(lr+lf)^2)+1;
    z4=(lr+lf)*sqrt(z3);
    z5=(lr+lf)^3*sqrt(z3^3);
    z6=(lr+lf)*z3;
    z7=v*(tan(delta)^2+1)*(1/z4-(lr*tan(delta))^2/z5);
    z8=T*lr*v*(tan(delta)^2+1);
    Ad=[1 0 -T*v*sin(z1) T*cos(z1)-(z2*sin(z1)/(2*z4));
        0 1 T*v*cos(z1)  T*sin(z1)+(z2*cos(z1)/(2*z4));
        0 0 1            (T*tan(delta))/z4        ;
        0 0 0            1 ;
    ];
    Bd=[(T^2*cos(z1)/2)    ,-(T^2*v*z7*sin(z1)/2)-(z8*sin(z1)/z6);
        (T^2*sin(z1))/2    ,(T^2*v*z7*cos(z1)/2)+(z8*cos(z1)/z6);
        T^2*tan(delta)/2*z4, T*z7;
        T                  ,0;
    ];
        y(1) = xmeasure(1)+T*ds+Ad(1,:)*[x(1)-xmeasure(1),x(2)-xmeasure(2),x(3)-xmeasure(3),x(4)-xmeasure(4)]'+Bd(1,:)*[u(1),u(2)]';
        y(2) = xmeasure(2)+T*dd+Ad(2,:)*[x(1)-xmeasure(1),x(2)-xmeasure(2),x(3)-xmeasure(3),x(4)-xmeasure(4)]'+Bd(2,:)*[u(1),u(2)]';   
        y(3) = xmeasure(3)+T*dphi+Ad(3,:)*[x(1)-xmeasure(1),x(2)-xmeasure(2),x(3)-xmeasure(3),x(4)-xmeasure(4)]'+Bd(3,:)*[u(1),u(2)]';
        y(4) = xmeasure(4)+T*dv+Ad(4,:)*[x(1)-xmeasure(1),x(2)-xmeasure(2),x(3)-xmeasure(3),x(4)-xmeasure(4)]'+Bd(4,:)*[u(1),u(2)]';
%     y=(xinital'+T*[ds,dd,dphi,dv]'+Ad*(x-xinital)'+Bd*u)';
%     if ~predictionflag
%         keyboard()
%     end
end

function [xcontinuos]=newIntegratedState(x,u,T) 
    lr=2;
    lf=2;
    v=x(:,4);
    phi=x(:,3);
    a=(u(1,:))';
    delta=(u(2,:))';
    alpha=atan((lr/(lr+lf))*tan(delta));
    xcontinuos(:,1)=v.*cos(phi + alpha);%longitudinal speed ds
    xcontinuos(:,2)=v.*sin(phi + alpha);%lateral speed dd
    xcontinuos(:,3)=(v/lr).*sin(alpha);%change in yaw angle(derivative) dphi
    xcontinuos(:,4)=a;%acceleration dv
end

function[ds,dd,dphi,dv]=kinematicbicycle(v,phi,a,delta)
    lr=2;
    lf=2;
    alpha=atan((lr/(lr+lf))*tan(delta));
    ds=v*cos(phi + alpha);
    dd=v*sin(phi + alpha);
    dphi=(v/lr)*sin(alpha); 
    dv=a;
end

function printHeader()
    fprintf('   k  |      u1(k)        u2(k)         x(1)         x(2)        x(3)         x(4)      Time\n');
    fprintf('--------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)
    fprintf(' %3d  | %+11.6f  %+11.6f  %+11.6f  %+11.6f  %+11.6f  %+11.6f  %+6.3f', mpciter, ...
            u(1,1)  , u(2,1) ,  x(1)  ,   x(2)  ,    x(3)  ,  x(4)  ,  t_Elapsed);
end

function plotTrajectories(dynamic, system, T, t0, x0, u, xmeasure, ...
                          atol_ode, rtol_ode, type )
   [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
                                  x0, u, xmeasure, atol_ode, rtol_ode, type);
                         
%     figure(1);
%         title('Speed vs lateral deviation closed loop trajectory');
%         xlabel('Speed(x_4)');
%         ylabel('Lateral Deviation(x_2)');
%         hold on
%         grid on
%         plot(linspace(40,40,5),linspace(-7,7,5),'y');
%         plot(linspace(-10,100,11),linspace(6,6,11),'b');
%         plot(linspace(-10,100,11),linspace(-6,-6,11),'b');
%         plot(x_intermediate(:,4),x_intermediate(:,2),'-or', ...
%              'MarkerFaceColor','r');
%         axis([0 40 -7 7]);
%         legend('Speed limit','Road Boundarie Right','Road Boundarie Left','Speed/Deviation Plot')
%     figure(1);
%         title('Longitudinal vs Lateral deviation closed loop trajectory');
%         xlabel('Longitudinal Deviation(x_1)');
%         ylabel('Lateral deviation(x_2)');
%         hold on
%         grid on
%         plot(linspace(0,100,10),linspace(6,6,10),'b');
%         plot(linspace(0,100,10),linspace(-6,-6,10),'b');
%         plot(linspace(0,0,10),linspace(-6,6,10),'g');
%         plot(x_intermediate(:,1),x_intermediate(:,2),'-or', ...
%              'MarkerFaceColor','r');
%         legend('Road Boundaries Right','Road Boundaries Left','Road Beggining','Trajectory');
%         axis([0 60 -7 7]);   
%     figure(2);
%         title(['x_1,x_2,x_3 and x_4 closed loop trajectory']);
%         xlabel('Iteration n');
%         ylabel('x_1(n),x_2(n),x_3(n),x_4(n)');
%         grid on;
%         hold on;
%         plot(t_intermediate*5,x_intermediate(:,1),'-ok','MarkerFaceColor','b');
%         plot(t_intermediate*5,x_intermediate(:,2),'-ok','MarkerFaceColor','g');
%         plot(t_intermediate*5,x_intermediate(:,3),'-ok','MarkerFaceColor','c');
%         plot(t_intermediate*5,x_intermediate(:,4),'-ok','MarkerFaceColor','y');
%         legend('x_1','x_2','x_3','x_4')
%         axis([0 30 -5 40]);
%         axis square;
%     figure(3)
%         title(['Longitudinal Deviation']);
%         xlabel('Time t(s)');
%         ylabel('Longitudinal Deviation d(m)');
%         hold on;
%         grid on;
%         plot(t_intermediate,x_intermediate(:,1),'-ok','MarkerFaceColor','b');
%     figure(4)
%         title(['Lateral Deviation']);
%         xlabel('Time t(s)');
%         ylabel('Lateral Deviation s(m)');
%         hold on;
%         grid on;
%         plot(t_intermediate,x_intermediate(:,2),'-ok','MarkerFaceColor','g');
%     figure(5)
%         title(['Yaw Angle']);
%         xlabel('Time t(s)');
%         ylabel('Yaw Angle \phi(rad)');
%         hold on;
%         grid on;
%         plot(t_intermediate,x_intermediate(:,3),'-ok','MarkerFaceColor','c');
%     figure(6)
%         title(['Velocity']);
%         xlabel('Time t(s)');
%         ylabel('Velocity v(m/s)');
%         hold on;
%         grid on;
%         plot(t_intermediate,x_intermediate(:,4),'-ok','MarkerFaceColor','y');
        
        
      
end

