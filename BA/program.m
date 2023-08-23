function program
    addpath('./urbannmpc');
    mpciterations = 150;
    N             = 10;
    T             = 0.2;
    tmeasure      = 0.0;
    xmeasure      = [60.2 0 0 3.3];% 43 -4 0 12 ,     40 -4 0 5 problem with 1st input its false but makes sense why it is false  60.2 0 0 3.3
    u0            = zeros(2,N);
    iprint        = 4;
    type          = 'difference equation';
    tol_opt       = 1e-8;
    opt_option    = 0;
    atol_ode_real = 1e-12;
    rtol_ode_real = 1e-12;
    atol_ode_sim  = 1e-4;
    rtol_ode_sim  = 1e-4;
    lr=2;
    lf=2;
    k=0;
    Q=diag([0,0.25,0,10]);
    R=diag([0.33,0]);%0.33,0.5
    zinital=[];% 60 5 -4 0; 82 5 -4 0 ,  100 5 -4 0 problem with 1st input
    m=size(zinital,1);
    zref=targetvehiclereference(zinital,m);
    pedinital=[80 0 -4 1];%90 0 -8 1  %60 0 -4 1.4  %  80 0 -4 1
    mped=size(pedinital,1);
    pedref=pedestrianreference(pedinital,mped);
    Tn= N*T;
    Qhl=diag([0,0.5]);
    Rhl=diag([5,0]);%problem with rhl(2)=0
    e0=[xmeasure(1,4);0]*ones(1,N);
    if mped~=0
        e0=[13;0]*ones(1,N);
    end
%     if mped==0 && m==2
%         e0=[13;4]*ones(1,N);
%     end

    
    
    
    
    function [zref]=targetvehiclereference(zinital,m)
        zref=zeros(m,4);
        for q=1:m
            if zinital(q,3)>-6.5 && zinital(q,3)<-2
                zref(q,:)=[0 5 -4 0];
                elseif zinital(q,3)>-2 && zinital(q,3)<2
                    zref(q,:)=[0 5 0 0];
                else
                    zref(q,:)=[0 5 4 0];
            end
        end
    end

    function [pedref]=pedestrianreference(pedinital,mped)
        pedref=zeros(mped,4);
        for q=1:mped
            pedref=[pedinital(q,1) 0 0 0.5];
        end
    end

% function [varykappa,varxkappa]=errorestimate(N,T,K)
%     Epsilon=diag([0.025,0.025,0.0028,0.0028]);%0.25 0.25 0.028 0.028
%     ZeroEpsilon=Epsilon;
%     EpsilonTV=diag([0.044, 0.009]);% 0.44 0.09
%     error(1,:)=(sqrt(ZeroEpsilon)*randn(4,1))';
%     A=[1 T 0 0;
%     0 1 0 0;
%     0 0 1 T;
%     0 0 0 1];
%     B=[0.5*T^2 0;  
%     T 0;
%     0 0.5*T^2;
%     0 T];
%     FullEpsilon=[];
%     for k=2:N+1 
%         NewEpsilon=(A+B*K)*Epsilon*(A+B*K)'+B*EpsilonTV*B';
%         FullEpsilon=[FullEpsilon;NewEpsilon];
%         error(k,:)=((A+B*K)*error(k-1,:)'+B*sqrt(EpsilonTV)*randn(2,1))';
%         Epsilon=NewEpsilon;
%     end
%     FullEpsilon=[ZeroEpsilon;FullEpsilon];
%     errorhat=[error(:,1),error(:,3)];
%     for m=0:N
%         varx(m+1,1)=FullEpsilon(1+4*m,1);
%         vary(m+1,1)=FullEpsilon(3+4*m,3);
%     end
%     beta=0.6;% betas 0.3 and 0.88 , Noises Epsilon=[3 3 1.25 1.25] EpsilonTV=[3 1.25] z=[100 20 -4 0; -40 20 0 0] x=[40 -4 0 30]
%     kappa=-2*log(1-beta);
%     varxkappa=sqrt(kappa)*varx;
%     varykappa=sqrt(kappa)*vary;
% end

%     function [z]=targetvehicle(T,mpciterations,zref,zinital,m) %z is the target vehicle state variable
%         A=[1 T 0 0;
%         0 1 0 0;
%         0 0 1 T;
%         0 0 0 1];
%         B=[0.5*T^2 0;  
%         T 0;
%         0 0.5*T^2;
%         0 T];
%         K12=-0.55;
%         K21=-0.63;
%         K22=-1;
%         K=[0 K12 0 0;
%             0 0 K21 K22];
%         z=zeros(mpciterations,4,m);
%         for q=1:m
%         z(1,:,q)=zinital(q,:)+sqrt((diag([0.25,0.25,0.028,0.028]))*rand(4,1))';
%         zinitialhat=z(1,:,q);
%             for iter=2:mpciterations
%                 temp=A*zinitialhat'+B*K*(zinitialhat-zref(q,:))'+B*sqrt(diag([0.44,0.09]))*randn(2,1);
%                 z(iter,:,q)=temp';
%                 zinitialhat=temp';
%             end
%         end
%     end

    [t, x, u] = urbannmpc(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system, ...
         mpciterations, N, T, tmeasure, xmeasure, u0,tol_opt, opt_option, ...
         type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim,iprint,@printHeader, @printClosedloopData,@plotTrajectories ...
          ,lr, lf , k, Q, R , zinital, m , zref , Tn, Qhl, Rhl,@runningcostsHL, @terminalcostsHL, @constraintsHL, ...
         @terminalconstraintsHL, @linearconstraintsHL, @systemHL,e0,pedinital,mped,pedref,@printClosedloopData2,@printHeader2);
    rmpath('./urbannmpc');
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = runningcosts(t, x, u, xref, Q, R)
    cost =(x-xref)*Q*(x-xref)'+u'*R*u;
end

function cost = terminalcosts(t, x, xref, Q, R)
    cost = 0.0;
end

function [c,ceq] = constraints(t, x, u, zpred, m, arvector, brvector,pedpred,mped,arvectorped,brvectorped,qx,qy,qt)%add mped to function
    minv = 0 ;
    maxv = 15;
    if mped==0
        roadwidth=8.2;%changed from 12?
    else
        roadwidth=2;
    end
    c(1) = x(2)-roadwidth/2;
    c(2) = -x(2)-roadwidth/2;
    c(3) = x(4)-maxv;
    c(4) = -x(4)-minv;
    if mped==0
        for i=1:m
            c(4+i) = 1-((x(1)-zpred(1,1,i))^2)/arvector^2-((x(2)-zpred(1,3,i))^2)/brvector^2;
        end
        for k=1:m
            c(4+m+k) = qx(:,k)*x(1)+qy(:,k)*x(2)+qt(:,k);
        end
    else
        for r=1:mped
            c(4+r) = 1-((x(1)-pedpred(1,1,r))^2)/arvectorped^2-((x(2)-pedpred(1,3,r))^2)/brvectorped^2;
        end
%         for k=1:mped
%             c(4+m+k) = qx(:,k)*x(1)+qy(:,k)*x(2)+qt(:,k);
%         end
     end
    ceq = [];
end

function [c,ceq] = terminalconstraints(~,~,~,~,~,~,~,~,~,~)
     c= [];
     ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [-9,-0.4];%-9
    ub  = [5,0.4];%5
end
    

function y = system(t, x, u, T, lr , lf , xmeasure , k)
[Ad, Bd, x0fcT] = computeCurvedBicycleModel(xmeasure, T, k, lf, lr);
% y=(x0fcT+Ad*[x(1)-xmeasure(1),x(2)-xmeasure(2),x(3)-xmeasure(3),x(4)-xmeasure(4)]'+Bd*[u(1);u(2)])';
y=(x0fcT+Ad*(x-xmeasure)'+Bd*u)';
end

function [Ad, Bd, x0fcT] = computeCurvedBicycleModel(x, T, k, lf, lr)

    d = x(2);
    v = x(4);
    c0 = cos(x(3));
    s0 = sin(x(3));
    alpha1 = lr/(lr+lf);
    
    w0 = 1-k*d;
    w1 = 1/w0;
    lambda = v*k*w1;

    z0 = sqrt(1-5*c0^2);  %   z1-z2=2*z0
    z1 = s0+z0;
    z2 = s0-z0;
    e1 = exp(lambda*z1*T/2);
    e2 = exp(lambda*z2*T/2);
    z3 = e1-e2;
    z4 = z1*e2-z2*e1;
    z5 = z1*e1-z2*e2;

    if abs(z0)<1e-5
        z6 = lambda*T;
        z7 = 2;
        z8 = 2*(1+lambda*s0*T);
        z9 = lambda*T^2/2;
        z10 = 2*T;
        z11 = 2*T+lambda*s0*T^2;
    else
        z6 = z3/z0;
        z7 = z4/z0;
        z8 = z5/z0;
        if lambda==0
            z9 = 0;
            z10 = 2*T;
            z11 = 2*T+lambda*s0*T^2;
        else
            if z1*z2==0
                if z1==0
                    z9 = (T-2*(e2-1)/lambda/z2)/z0;
                    z10 = -z2/z0*T;
                else
                    z9 = (2*(e1-1)/lambda/z1-T)/z0;
                    z10 = z1/z0*T;
                end
            else
                z9 = 2*((e1-1)/z1-(e2-1)/z2)/z0/lambda;
                z10 = 2*((e2-1)*z1/z2-(e1-1)*z2/z1)/z0/lambda;
            end
            z11 = 2*z6/lambda;
        end
    end

    if lambda==0
        a13 = -v*s0*T;
        a14 = T*c0*w1;
        a23 = v*c0*T;
        a24 = T*s0;
        a34 = -k*c0*T*w1;
        b11 = c0*T^2/2*w1;
        b21 = s0*T^2/2;
        b31 = -k*T^2*c0/2*w1;
        b12 = -(1+v*T/2/lr)*v*alpha1*s0*T*w1;
        b22 = (1+v*T/2/lr)*v*T*c0*alpha1;
        b32 = v*alpha1*T/lr;
    else
        if c0==0
            a14 = 0;
            a24 = s0*T;
            a34 = 0;
            b11 = 0;
            b21 = s0*T^2/2;
            b31 = 0;
        else
            a14 = (s0*(1-z8/2)+z6)/(v*k*c0);
            a24 = (-1+s0*c0^2*z6+z7/2)/lambda/c0^2;
            a34 = (s0*(z8/2-1)-z6)/v/c0;
            b11 = (s0*(T-z11/2)+z9)/(v*k*c0);
            b21 = (-T+s0*c0^2*z9+z10/2)/lambda/c0^2;
            b31 = (s0*(z11/2-T)-z9)/v/c0;
        end
        a13 = (1-z8/2)/k;
        a23 = w0*c0*z6/k;
        w2 = (w0/k/lr+s0);
        b12 = (-T*s0+c0^2*z9+(T-z11/2)*w2)*v*alpha1*w1;
        b22 = (z10/2+z9*w2)*v*c0*alpha1;
        b32 = (z11*w2/2-z9*c0^2)*lambda*alpha1;
    end

    Ad = [1,    c0*w1*z6,    a13,         a14;
          0,    z7/2,             a23,         a24;
          0,    -k*c0*z6*w1, z8/2,   a34;
          0,    0,                    0,          1];

    Bd = [b11,   b12;
          b21,   b22;
          b31,   b32;
          T,    0];
      
    q1 = 1/(1-k*x(2));    %   1/(1-k*d)
    q2 = cos(x(3));        %   cos(phi)
    q3 = x(4)*q2*q1;       %   s_dot = v*cos(phi)/(1-k*d)
    fcq = [q3; x(4)*sin(x(3)); -k*q3; 0];
    x0fcT = x'+T*fcq;
end



function printHeader()
    fprintf('   k  |      u1(k)        u2(k)         x(1)         x(2)        x(3)         x(4)      Time\n');
    fprintf('--------------------------------------------------\n');
end

function printHeader2()
    fprintf('   k  |      e1(k)        e2(k)         p(1)         p(2)        Time\n');
    fprintf('--------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)
    fprintf(' %3d  | %+11.6f  %+11.6f  %+11.6f  %+11.6f  %+11.6f  %+11.6f  %+6.3f', mpciter, ...
            u(1,1)  , u(2,1) ,  x(1)  ,   x(2)  ,    x(3)  ,  x(4)  ,  t_Elapsed);
end

function printClosedloopData2(mpciter, e, p, t_ElapsedHL)
    fprintf(' %3d  | %+11.6f  %+11.6f  %+11.6f  %+11.6f   %+6.3f', mpciter, ...
            e(1,1)  , e(2,1) ,  p(1)  ,   p(2)  ,  t_ElapsedHL);
end



function plotTrajectories(dynamic, system, T, t0, x0, u, ...
                          atol_ode, rtol_ode, type )
   [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
                                  x0, u, atol_ode, rtol_ode, type);
        
        
      
end


function costHL = runningcostsHL(t, p , e, pref,eref, Qhl, Rhl)
    costHL =(p-pref)*Qhl*(p-pref)'+(e-eref)'*Rhl*(e-eref);% what to use instead of xref Q? and R change x and u with p and e ,penilize speed more
end

function costHL = terminalcostsHL(t, p, pref, eref, Qhl, Rhl)
    costHL =0.0 ;%dont use  (p-pref)*Qhl*(p-pref)'
end

function [c,ceq] = constraintsHL(t, p, e, zpredHL,m,arvectorHL,brvectorHL,pedpredHL,mped,arvectorpedHL,brvectorpedHL,eprevious)% add mped and vectors
    if mped==0
        roadwidth=8.2;%changed from 12? speed problem with vehicle simulation
        maxdelta=1000;
    else
        roadwidth=0.1;
        maxdelta=4;
    end
%     length=15;
%     width=3;
%     maxdelta=4;
%     mindelta=-3;
    c(1) = p(2)-roadwidth/2;
    c(2) = -p(2)-roadwidth/2;
    c(3) = e(1)-eprevious(1)-maxdelta;%works only in pedestrian case for some reason
    c(4) = eprevious(1)-e(1)-maxdelta;
    if mped==0
        for i=1:m
            c(4+i) = 1-((p(1)-zpredHL(1,1,i))^2)/arvectorHL^2-((p(2)-zpredHL(1,3,i))^2)/brvectorHL^2;
        end
    else
        for r=1:mped
            c(4+r) = 1-((p(1)-pedpredHL(1,1,r))^2)/arvectorpedHL^2-((p(2)-pedpredHL(1,3,r))^2)/brvectorpedHL^2;
        end
    end
    ceq = [];
end

function [c,ceq] = terminalconstraintsHL(t, p, zpredHL,m,arvectorHL,brvectorHL,pedpredHL,mped,arvectorpedHL,brvectorpedHL)
%     if mped==0
%         roadwidth=8.2;%changed from 12?
%     else
%         roadwidth=1;
%     end
% %     length=15;
% %     width=3;
%     c(1) = p(2)-roadwidth/2;
%     c(2) = -p(2)-roadwidth/2;
%     for i=1:m
%         c(2+i) = 1-((p(1)-zpredHL(1,1,i))^2)/arvectorHL^2-((p(2)-zpredHL(1,3,i))^2)/brvectorHL^2;
%     end
%     for r=1:mped
%         c(2+m+r) = 1-((p(1)-pedpredHL(1,1,r))^2)/arvectorpedHL^2-((p(2)-pedpredHL(1,3,r))^2)/brvectorpedHL^2;
%     end
     c   = [];
     ceq = [];
end% stays same except for change x and u with p and e

function [A, b, Aeq, beq, lb, ub] = linearconstraintsHL(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [0,-4];
    ub  = [15,4];
end%1st constraint is for speed 2nd is for delta(what is delta how to calculate it)
    

function y = systemHL(t, p, e, Tn)
y(1)=p(1)+Tn*e(1);
y(2)=p(2)+e(2);
end%system is given in notes. what is delta?
