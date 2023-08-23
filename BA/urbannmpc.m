function [t, x, u] = urbannmpc(runningcosts, terminalcosts, ...
              constraints, terminalconstraints, ...
              linearconstraints, system, ...
              mpciterations, N, T, tmeasure, xmeasure, u0, ...
              varargin)
% nmpc(runningcosts, terminalcosts, constraints, ...
%      terminalconstraints, linearconstraints, system, ...
%      mpciterations, N, T, tmeasure, xmeasure, u0, ...
%      tol_opt, opt_option, ...
%      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
%      iprint, printHeader, printClosedloopData, plotTrajectories)
% Computes the closed loop solution for the NMPC problem defined by
% the functions
%   runningcosts:         evaluates the running costs for state and control
%                         at one sampling instant.
%                         The function returns the running costs for one
%                         sampling instant.
%          Usage: [cost] = runningcosts(t, x, u)
%                 with time t, state x and control u
%   terminalcosts:        evaluates the terminal costs for state at the end
%                         of the open loop horizon.
%                         The function returns value of the terminal costs.
%          Usage: cost = terminalcosts(t, x)
%                 with time t and state x
%   constraints:          computes the value of the restrictions for a
%                         sampling instance provided the data t, x and u
%                         given by the optimization method.
%                         The function returns the value of the
%                         restrictions for a sampling instance separated
%                         for inequality restrictions c and equality
%                         restrictions ceq.
%          Usage: [c,ceq] = constraints(t, x, u)
%                 with time t, state x and control u
%   terminalconstraints:  computes the value of the terminal restrictions
%                         provided the data t, x and u given by the
%                         optimization method.
%                         The function returns the value of the
%                         terminal restriction for inequality restrictions
%                         c and equality restrictions ceq.
%          Usage: [c,ceq] = terminalconstraints(t, x)
%                 with time t and state x
%   linearconstraints:    sets the linear constraints of the discretized
%                         optimal control problem. This is particularly
%                         useful to set control and state bounds.
%                         The function returns the required matrices for
%                         the linear inequality and equality constraints A
%                         and Aeq, the corresponding right hand sides b and
%                         beq as well as the lower and upper bound of the
%                         control.
%          Usage: [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
%                 with time t, state x and control u
%   system:               evaluates the difference equation describing the
%                         process given time t, state vector x and control
%                         u.
%                         The function returns the state vector x at the
%                         next time instant.
%          Usage: [y] = system(t, x, u, T)
%                 with time t, state x, control u and sampling interval T
% for a given number of NMPC iteration steps (mpciterations). For
% the open loop problem, the horizon is defined by the number of
% time instances N and the sampling time T. Note that the dynamic
% can also be the solution of a differential equation. Moreover, the
% initial time tmeasure, the state measurement xmeasure and a guess of
% the optimal control u0 are required.
%
% Arguments:
%   mpciterations:  Number of MPC iterations to be performed
%   N:              Length of optimization horizon
%   T:              Sampling interval
%   tmeasure:       Time measurement of initial value
%   xmeasure:       State measurement of initial value
%   u0:             Initial guess of open loop control
%
% Optional arguments:
%       iprint:     = 0  Print closed loop data(default)
%                   = 1  Print closed loop data and errors of the
%                        optimization method
%                   = 2  Print closed loop data and errors and warnings of
%                        the optimization method
%                   >= 5 Print closed loop data and errors and warnings of
%                        the optimization method as well as graphical
%                        output of closed loop state trajectories
%                   >=10 Print closed loop data and errors and warnings of
%                        the optimization method with error and warning
%                        description
%   printHeader:         Clarifying header for selective output of closed
%                        loop data, cf. printClosedloopData
%   printClosedloopData: Selective output of closed loop data
%   plotTrajectories:    Graphical output of the trajectories, requires
%                        iprint >= 4
%       tol_opt:         Tolerance of the optimization method
%       opt_option: = 0: Active-set method used for optimization (default)
%                   = 1: Interior-point method used for optimization
%                   = 2: Trust-region reflective method used for
%                        optimization
%   type:                Type of dynamic, either difference equation or
%                        differential equation can be used
%    atol_ode_real:      Absolute tolerance of the ODE solver for the
%                        simulated process
%    rtol_ode_real:      Relative tolerance of the ODE solver for the
%                        simulated process
%    atol_ode_sim:       Absolute tolerance of the ODE solver for the
%                        simulated NMPC prediction
%    rtol_ode_sim:       Relative tolerance of the ODE solver for the
%                        simulated NMPC prediction
%
% Internal Functions:
%   measureInitialValue:          measures the new initial values for t0
%                                 and x0 by adopting values computed by
%                                 method applyControl.
%                                 The function returns new initial state
%                                 vector x0 at sampling instant t0.
%   applyControl:                 applies the first control element of u to
%                                 the simulated process for one sampling
%                                 interval T.
%                                 The function returns closed loop state
%                                 vector xapplied at sampling instant
%                                 tapplied.
%   shiftHorizon:                 applies the shift method to the open loop
%                                 control in order to ease the restart.
%                                 The function returns a new initial guess
%                                 u0 of the control.
%   solveOptimalControlProblem:   solves the optimal control problem of the
%                                 horizon N with sampling length T for the
%                                 given initial values t0 and x0 and the
%                                 initial guess u0 using the specified
%                                 algorithm.
%                                 The function returns the computed optimal
%                                 control u, the corresponding value of the
%                                 cost function V as well as possible exit
%                                 flags and additional output of the
%                                 optimization method.
%   costfunction:                 evaluates the cost function of the
%                                 optimal control problem over the horizon
%                                 N with sampling time T for the current
%                                 data of the optimization method t0, x0
%                                 and u.
%                                 The function return the computed cost
%                                 function value.
%   nonlinearconstraints:         computes the value of the restrictions
%                                 for all sampling instances provided the
%                                 data t0, x0 and u given by the
%                                 optimization method.
%                                 The function returns the value of the
%                                 restrictions for all sampling instances
%                                 separated for inequality restrictions c
%                                 and equality restrictions ceq.
%   computeOpenloopSolution:      computes the open loop solution over the
%                                 horizon N with sampling time T for the
%                                 initial values t0 and x0 as well as the
%                                 control u.
%                                 The function returns the complete open
%                                 loop solution over the requested horizon.
%   dynamic:                      evaluates the dynamic of the system for
%                                 given initial values t0 and x0 over the
%                                 interval [t0, tf] using the control u.
%                                 The function returns the state vector x
%                                 at time instant tf as well as an output
%                                 of all intermediate evaluated time
%                                 instances.
%   printSolution:                prints out information on the current MPC
%                                 step, in particular state and control
%                                 information as well as required computing
%                                 times and exitflags/outputs of the used
%                                 optimization method. The flow of
%                                 information can be controlled by the
%                                 variable iprint and the functions
%                                 printHeader, printClosedloopData and
%                                 plotTrajectories.
%
% Version of May 30, 2011, in which a bug appearing in the case of 
% multiple constraints has been fixed
%
% (C) Lars Gruene, Juergen Pannek 2011

    if (nargin>=13)
        tol_opt = varargin{1};
    else
        tol_opt = 1e-6;
    end;
    if (nargin>=14)
        opt_option = varargin{2};
    else
        opt_option = 0;
    end;
    if (nargin>=15)
        if ( strcmp(varargin{3}, 'difference equation') || ...
                strcmp(varargin{3}, 'differential equation') )
            type = varargin{3};
        else
            fprintf([' Wrong input for type of dynamic: use either ', ...
                '"difference equation" or "differential equation".']);
        end
    else
        type = 'difference equation';
    end;
    if (nargin>=16)
        atol_ode_real = varargin{4};
    else
        atol_ode_real = 1e-8;
    end;
    if (nargin>=17)
        rtol_ode_real = varargin{5};
    else
        rtol_ode_real = 1e-8;
    end;
    if (nargin>=18)
        atol_ode_sim = varargin{6};
    else
        atol_ode_sim = atol_ode_real;
    end;
    if (nargin>=19)
        rtol_ode_sim = varargin{7};
    else
        rtol_ode_sim = rtol_ode_real;
    end;
    if (nargin>=20)
        iprint = varargin{8};
    else
        iprint = 0;
    end;
    if (nargin>=21)
        printHeader = varargin{9};
    else
        printHeader = @printHeaderDummy;
    end;
    if (nargin>=22)
        printClosedloopData = varargin{10};
    else
        printClosedloopData = @printClosedloopDataDummy;
    end;
    if (nargin>=23)
        plotTrajectories = varargin{11};
    else
        plotTrajectories = @plotTrajectoriesDummy;
    end;
    lr=varargin{12};
    lf=varargin{13};
    k=varargin{14};
    Q=varargin{15};
    R=varargin{16};
    zinital=varargin{17};
    m=varargin{18};
    zref=varargin{19};
    Tn=varargin{20};
    Qhl=varargin{21};
    Rhl=varargin{22};
    runningcostsHL=varargin{23};
    terminalcostsHL=varargin{24};
    constraintsHL=varargin{25};
    terminalconstraintsHL=varargin{26};
    linearconstraintsHL=varargin{27};
    systemHL=varargin{28};
    e0=varargin{29}; 
    pedinital=varargin{30};
    mped=varargin{31};
    pedref=varargin{32};
    if (nargin>=45)
        printClosedloopData2 = varargin{33};
    else
        printClosedloopData2 = @printClosedloopData2Dummy;
    end
    if (nargin>=46)
        printHeader2 = varargin{34};
    else
        printHeader2 = @printHeader2Dummy;
    end
    
%     lr=varargin{11};
%     lf=varargin{12};
%     k=varargin{13};
%     Q=varargin{14};
%     R=varargin{15};
%     zinital=varargin{16};
%     m=varargin{17};
%     zref=varargin{18};
%     z=varargin{19};
     
    

    % Determine MATLAB Version and
    % specify and configure optimization method
    vs = version('-release');
    vyear = str2num(vs(1:4));
    if (vyear <= 2007)
        fprintf('MATLAB version R2007 or earlier detected\n');
        if ( opt_option == 0 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 2000,...
                'LargeScale', 'off',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1);
        elseif ( opt_option == 1 )
            error('nmpc:WrongArgument', '%s\n%s', ...
                  'Interior point method not supported in MATLAB R2007', ...
                  'Please use opt_option = 0 or opt_option = 2');
        elseif ( opt_option == 2 )
             options = optimset('Display','off',...
                 'TolFun', tol_opt,...
                 'MaxIter', 2000,...
                 'LargeScale', 'on',...
                 'Hessian', 'off',...
                 'MaxPCGIter', max(1,floor(size(u0,1)*size(u0,2)/2)),...
                 'PrecondBandWidth', 0,...
                 'TolPCG', 1e-1);
        end
    else
        fprintf('MATLAB version R2008 or newer detected\n');
        if ( opt_option == 0 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);
        elseif ( opt_option == 1 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 2000,...
                'Algorithm', 'interior-point',...
                'AlwaysHonorConstraints', 'bounds',...
                'FinDiffType', 'forward',...
                'HessFcn', [],...
                'Hessian', 'bfgs',...
                'HessMult', [],...
                'InitBarrierParam', 0.1,...
                'InitTrustRegionRadius', sqrt(size(u0,1)*size(u0,2)),...
                'MaxProjCGIter', 2*size(u0,1)*size(u0,2),...
                'ObjectiveLimit', -1e20,...
                'ScaleProblem', 'obj-and-constr',...
                'SubproblemAlgorithm', 'cg',...
                'TolProjCG', 1e-2,...
                'TolProjCGAbs', 1e-10);
        %                       'UseParallel','always',...
        elseif ( opt_option == 2 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 2000,...
                'Algorithm', 'trust-region-reflective',...
                'Hessian', 'off',...
                'MaxPCGIter', max(1,floor(size(u0,1)*size(u0,2)/2)),...
                'PrecondBandWidth', 0,...
                'TolPCG', 1e-1);
        end
    end

    warning off all
    t = [];
    x = [];
    u = [];
    p = [];%state reduced
    e = [];%input reduced xmeasure(4); 0
    K=[0 -0.55 0 0;
       0 0 -0.63 -0.95];
    vehiclelength=5;
    vehiclewidth=2;
    safefactor=0.5;
    pedlength=1;
    pedwidth=1;
    safetywidth=0;
    initalnoise=diag([0.25,0.25,0.028,0.028]);%0.25,0.25,0.028,0.028
    inputnoise=diag([0.44,0.09]);%0.44,0.09
    [varykappaHL,varxkappaHL]=errorestimate(N,Tn,K,0.01*initalnoise,0.01*inputnoise);%,initalnoise,inputnoise
    [arvectorHL,brvectorHL]=arandbr(varykappaHL,varxkappaHL,vehiclelength,vehiclewidth,2.5*safefactor);
    [varykappa,varxkappa]=errorestimate(N,T,K,0.01*initalnoise,0.01*inputnoise);
    [arvector,brvector]=arandbr(varykappa,varxkappa,vehiclelength,vehiclewidth,2.5*safefactor);
    [varykappapedHL,varxkappapedHL]=errorestimate(N,Tn,0*K,0*initalnoise,0*inputnoise);
    [arvectorpedHL,brvectorpedHL]=arandbr(varykappapedHL,varxkappapedHL,pedlength,pedwidth,safefactor);
    [varykappaped,varxkappaped]=errorestimate(N,T,0*K,0*initalnoise,0*inputnoise);%,0.1*initalnoise,0.1*inputnoise
    [arvectorped,brvectorped]=arandbr(varykappaped,varxkappaped,pedlength,pedwidth,safefactor);
        if mped==0
        pref=[0 -4];
            else
            pref=[0 0];
        end
    eref=[12, 0]';%12 for vehicle, 10 for ped(doesnt matter acctually)
    amin=-9;
%     if mped~=0
%         eref=[12, 0];
%     end% 12 0 pedestrian situation
    % Start of the NMPC iteration
    mpciter = 0;
    
    while(mpciter < mpciterations)
        % Step (1) of the NMPC algorithm:
        %   Obtain new initial value
        [t0, x0] = measureInitialValue ( tmeasure, xmeasure );
%         if mpciter>14
%             zref(2,:)=[0 0 -4 1.7];
%         end
%         if mpciter>18
%             zref(2,:)=[0 0 0 0];
%         end
        [z]=targetvehicle(T,mpciterations,zref,zinital,m,K,0.01*initalnoise,0.01*inputnoise);
        [ped]=targetvehicle(T,mpciterations,pedref,pedinital,mped,0*K,0*initalnoise,0*inputnoise);
        
        if (mod(mpciter,N)==0)%change 10 with N
%             if mped~=0 &&  mpciter==10
%                 e0=[0;0]*ones(1,N);
%             end
            [p0]=reducedstate(x0,N);%p is new state variable b is the new input    
            pedpredHL=targetvehicleprediction(Tn,N,mpciter,ped,pedref,mped,0*K);
            zpredHL=targetvehicleprediction(Tn,N,mpciter,z,zref,m,K);
            t_StartHL=tic;
            [e_new,~,exitflagHL,outputHL,pstate] = solveOptimalControlProblemHL ...
            (runningcostsHL, terminalcostsHL, constraintsHL, ...
            terminalconstraintsHL, linearconstraintsHL, systemHL, ...
            N, t0, p0, e0, Tn, pref,eref, Qhl, Rhl,zpredHL,m,arvectorHL, brvectorHL, ...
            pedpredHL,mped,arvectorpedHL,brvectorpedHL,[x0(4);0],....
            atol_ode_sim, rtol_ode_sim, tol_opt, options, type);
            t_ElapsedHL = toc( t_StartHL );%e(1) and p(2) are used as reference ,[x0(4);0]
            drawrectanglesHL(p0,m,mped,zpredHL,pedpredHL,pstate,N,arvectorHL,brvectorHL,arvectorpedHL,brvectorpedHL);
            [~,p0]=applyControlHL(systemHL, Tn, t0, p0, e_new, atol_ode_real, rtol_ode_real, type);
            if ( iprint >= 1 )
            printSolution(systemHL, printHeader, printClosedloopData2, ...
                          plotTrajectories, mpciter, Tn, t0, p0, e_new, ...
                          atol_ode_sim, rtol_ode_sim, type, iprint, ...
                          exitflagHL, outputHL,t_ElapsedHL);
            end
            p0=roundup(p0);
            p=[p; p0];
            e=[e,e_new(:,1)]; %do i take first or second element of optimal control input
            e0=shiftHorizon(e_new);         
            xref=[0, p0(2), 0, e_new(1,1)];%e_new(1,1)
        end
        pedpred=targetvehicleprediction(T,N,mpciter,ped,pedref,mped,0*K);
        zpred=targetvehicleprediction(T,N,mpciter,z,zref,m,K);
        [xstop]=stoppingtime(x0,amin,N,T);
        [laneEV,laneTV,lanePED]=lanecheck(x0,zpred,m,mped,pedpred);
        [qx,qy,qt]=calculateQs(arvector,brvector,xstop,x0,m,safetywidth,laneEV,laneTV,N,zpred,arvectorped,mped,lanePED,pedpred);
%         [xref]=computeReference(x0);       

        
        % Step (2) of the NMPC algorithm:
        %   Solve the optimal control problem
        t_Start = tic;        
        [u_new, ~, exitflag , output] = solveOptimalControlProblem ...
            (runningcosts, terminalcosts, constraints, ...
            terminalconstraints, linearconstraints, system, ...
            N, t0, x0, u0, T, xref, Q, R, lr ,lf ,xmeasure,k,zpred,m,arvector,brvector,pedpred,mped,arvectorped,brvectorped,qx,qy,qt, ...
            atol_ode_sim, rtol_ode_sim, tol_opt, options, type);  % V current, output  
        t_Elapsed = toc( t_Start );
        %   Print solution
        if ( iprint >= 1 )
            printSolution(system, printHeader, printClosedloopData, ...
                          plotTrajectories, mpciter, T, t0, x0, u_new, ...
                          atol_ode_sim, rtol_ode_sim, type, iprint, ...
                          exitflag, output, t_Elapsed);
        end
        %   Store closed loop data
        t = [ t; tmeasure ];
        x = [ x; xmeasure ];
        u = [ u, u_new(:,1) ]; %change here
        %   Prepare restart
        u0 = shiftHorizon(u_new);
        % Step (3) of the NMPC algorithm:
        %   Apply control to process
        [tmeasure, xmeasure] = applyControl(system, T, t0, x0, u_new, lr ,lf ,xmeasure, k , ...
            atol_ode_real, rtol_ode_real, type);
        xstate=computeOpenloopSolution(system, N, T, t0, x0, u0, lr ,lf ,xmeasure,k,atol_ode_sim, rtol_ode_sim, type);%question do i write this after step 3 or before step 2 of the algorithm do i draw the guess for u0 or do i draw the first solution for u
        drawrectangles(x0,m,mped,zpred,pedpred,xstate,N,arvector,brvector,arvectorped,brvectorped,qx,qy,qt);
       
        mpciter = mpciter+1;
    end
end

function [t0, x0] = measureInitialValue ( tmeasure, xmeasure )
    t0 = tmeasure;
    x0 = xmeasure;
end

function [tapplied, xapplied] = applyControl(system, T, t0, x0, u, lr , lf , xmeasure, k, ...
                                atol_ode_real, rtol_ode_real, type)
    xapplied = dynamic(system, T, t0, x0, u(:,1), lr , lf , xmeasure,k, ...
                       atol_ode_real, rtol_ode_real, type);
    tapplied = t0+T;
end

function [tnapplied, papplied] = applyControlHL(systemHL, Tn, t0, p0, e, ...
                                atol_ode_real, rtol_ode_real, type)
    papplied = dynamicHL(systemHL, Tn, t0, p0, e(:,1), ...
                       atol_ode_real, rtol_ode_real, type);
    tnapplied = t0+Tn;
end

function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end

function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, constraints, terminalconstraints, ...
    linearconstraints, system, N, t0, x0, u0, T,xref, Q, R, lr ,lf ,xmeasure,k,zpred,m, arvector,brvector,pedpred,mped,arvectorped,brvectorped,qx,qy,qt,...
    atol_ode_sim, rtol_ode_sim, tol_opt, options, type)%N, t0, x0, u0, T, xref, Q, R, lr ,lf ,xmeasure,k,zpred,m,arvector,brvector,pedpred,mped,arvectorped,brvectorped,
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u0, lr ,lf ,xmeasure,k, ...
                                atol_ode_sim, rtol_ode_sim, type);

    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    for i=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+i*T,x(i,:),u0(:,i));
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end

    % Solve optimization problem
    [u, V, exitflag, output] = fmincon(@(u) costfunction(runningcosts, ...
        terminalcosts, system, N, T, t0, x0, ...
        u, xref, Q, R,lr , lf , xmeasure ,k, atol_ode_sim, rtol_ode_sim, type), u0, A, b, Aeq, beq, lb, ...
        ub, @(u) nonlinearconstraints(constraints, terminalconstraints, ...
        system, N, T, t0, x0, u,lr , lf , xmeasure ,k, zpred, m, arvector, brvector,pedpred,mped,arvectorped,brvectorped,qx,qy,qt,...
        atol_ode_sim, rtol_ode_sim, type), options);
end

function cost = costfunction(runningcosts, terminalcosts, system, ...
                    N, T, t0, x0, u, xref, Q, R,lr , lf , xmeasure ,k, ...
                    atol_ode_sim, rtol_ode_sim, type)
    cost = 0;
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u,lr , lf , xmeasure ,k, ...
                                atol_ode_sim, rtol_ode_sim, type);
    for i=1:N
        cost = cost+runningcosts(t0+i*T, x(i,:), u(:,i), xref, Q, R);
    end
    cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:),xref, Q, R);
end

function [c,ceq] = nonlinearconstraints(constraints, ...
    terminalconstraints, system, ...
    N, T, t0, x0, u,lr , lf , xmeasure ,k,zpred, m ,arvector, brvector,pedpred,mped,arvectorped,brvectorped,qx,qy,qt,atol_ode_sim, rtol_ode_sim, type)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u,lr , lf , xmeasure ,k, ...
                                atol_ode_sim, rtol_ode_sim, type);
    c = [];
    ceq = [];
    for i=2:N %change here
        [cnew, ceqnew] = constraints(t0+i*T,x(i,:),u(:,i),zpred(i,:,:),m,arvector(i),brvector(i),pedpred(i,:,:),mped,arvectorped(i),brvectorped(i),qx(i,:),qy(i,:),qt(i,:));%zpred(i,:,:) or zpred?
        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    [cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:),zpred(N+1,:,:),m,arvector(N+1),brvector(N+1),pedpred(N+1,:,:),mped,arvectorped(N+1),brvectorped(N+1));%zpred(i,:,:) or zpred?
    c = [c cnew];
    ceq = [ceq ceqnew];
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u, lr , lf , xmeasure ,k,...
                                     atol_ode_sim, rtol_ode_sim, type)
    x(1,:) = x0;
    for i=1:N
        x(i+1,:) = dynamic(system, T, t0, x(i,:), u(:,i), lr , lf , xmeasure ,k,...
                             atol_ode_sim, rtol_ode_sim, type);
    end
end

function [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
             x0, u, lr ,lf , xmeasure,k,atol_ode, rtol_ode, type)
    if ( strcmp(type, 'difference equation') )
        x = system(t0, x0, u, T, lr ,lf ,xmeasure,k);
        x_intermediate = [x0; x];
        t_intermediate = [t0, t0+T];
    elseif ( strcmp(type, 'differential equation') )
        options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);
        [t_intermediate,x_intermediate] = ode45(system, ...
            [t0, t0+T], x0, options, u);
        x = x_intermediate(size(x_intermediate,1),:);
    end
end

function [p, t_intermediate, p_intermediate] = dynamicHL(systemHL, Tn, t0, ...
             p0, e, atol_ode, rtol_ode, type)
    if ( strcmp(type, 'difference equation') )
        p = systemHL(t0, p0, e, Tn);
        p_intermediate = [p0; p];
        t_intermediate = [t0, t0+Tn];
    elseif ( strcmp(type, 'differential equation') )
        options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);
        [t_intermediate,p_intermediate] = ode45(systemHL, ...
            [t0, t0+Tn], p0, options, e);
        p = p_intermediate(size(p_intermediate,1),:);
    end
end



function printSolution(system, printHeader, printClosedloopData, ...
             plotTrajectories, mpciter, T, t0, x0, u, ...
             atol_ode, rtol_ode, type, iprint, exitflag, output, t_Elapsed)
    if (mpciter == 0)
        printHeader();
    end
    printClosedloopData(mpciter, u, x0, t_Elapsed);
    switch exitflag
        case -2
        if ( iprint >= 1 && iprint < 10 )
            fprintf(' Error F\n');
        elseif ( iprint >= 10 )
            fprintf(' Error: No feasible point was found\n')
        end
        case -1
        if ( iprint >= 1 && iprint < 10 )
            fprintf(' Error OT\n');
        elseif ( iprint >= 10 )
            fprintf([' Error: The output function terminated the',...
                     ' algorithm\n'])
        end
        case 0
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning IT\n');
        elseif ( iprint >= 10 )
            fprintf([' Warning: Number of iterations exceeded',...
                     ' options.MaxIter or number of function',...
                     ' evaluations exceeded options.FunEvals\n'])
        end
        case 1
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' \n');
        elseif ( iprint >= 10 )
            fprintf([' First-order optimality measure was less',...
                     ' than options.TolFun, and maximum constraint',...
                     ' violation was less than options.TolCon\n'])
        end
        case 2
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning TX\n');
        elseif ( iprint >= 10 )
            fprintf(' Warning: Change in x was less than options.TolX\n')
        end
        case 3
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning TJ\n');
        elseif ( iprint >= 10 )
            fprintf([' Warning: Change in the objective function',...
                     ' value was less than options.TolFun\n'])
        end
        case 4
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning S\n');
        elseif ( iprint >= 10 )
            fprintf([' Warning: Magnitude of the search direction',...
                     ' was less than 2*options.TolX and constraint',...
                     ' violation was less than options.TolCon\n'])
        end
        case 5
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning D\n');
        elseif ( iprint >= 10 )
            fprintf([' Warning: Magnitude of directional derivative',...
                     ' in search direction was less than',...
                     ' 2*options.TolFun and maximum constraint',...
                     ' violation was less than options.TolCon\n'])
        end
    end
    if ( iprint >= 5 )
        plotTrajectories(@dynamic, system, T, t0, x0, u, atol_ode, rtol_ode, type)
    end
end

function printHeaderDummy(varargin)
end

function printHeader2Dummy(varargin)
end

function printClosedloopDataDummy(varargin)
end

function printClosedloopData2Dummy(varargin)
end

function plotTrajectoriesDummy(varargin)
end

function [z]=targetvehicle(T,mpciterations,zref,zinital,m,K,initalnoise,inputnoise) %z is the target vehicle state variable
        A=[1 T 0 0;
        0 1 0 0;
        0 0 1 T;
        0 0 0 1];
        B=[0.5*T^2 0;  
        T 0;
        0 0.5*T^2;
        0 T];
        z=zeros(mpciterations,4,m);
        for q=1:m
        z(1,:,q)=zinital(q,:)+(sqrt(initalnoise)*randn(4,1))';%(sqrt(initalnoise*rand(4,1)))'   (sqrt(initalnoise)*randn(4,1))'
        zinitialhat=z(1,:,q);
            for iter=2:mpciterations
                temp=A*zinitialhat'+B*K*(zinitialhat-zref(q,:))'+B*sqrt(inputnoise)*randn(2,1);% should sqrt be for both randn and inputnoise?
                z(iter,:,q)=temp';
                zinitialhat=temp';
            end
        end
end

% function [z]=targetvehicle(T,mpciterations,zref,zinital,m,K) %z is the target vehicle state variable
% A=[1 T 0 0;
% 0 1 0 0;
% 0 0 1 T;
% 0 0 0 1];
% B=[0.5*T^2 0;  
% T 0;
% 0 0.5*T^2;
% 0 T];
% z=zeros(mpciterations,4,m);
% for k=1:m
% z(1,:,k)=zinital(k,:)+sqrt((diag([0.25,0.25,0.028,0.028]))*rand(4,1))';
% zinitialhat=z(1,:,k);
% for iter=2:mpciterations
%     temp=A*zinitialhat'+B*K*(zinitialhat-zref(k,:))'+B*sqrt(diag([0.44,0.09]))*randn(2,1);
%     z(iter,:,k)=temp';
%     zinitialhat=temp';
% end
% end
% end



function [varykappa,varxkappa]=errorestimate(N,T,K,initalnoise,inputnoise)
    ZeroEpsilon=initalnoise;
    error(1,:)=(sqrt(ZeroEpsilon)*randn(4,1))';
    A=[1 T 0 0;
    0 1 0 0;
    0 0 1 T;
    0 0 0 1];
    B=[0.5*T^2 0;  
    T 0;
    0 0.5*T^2;
    0 T];
    FullEpsilon=[];
    for k=2:N+1 
        NewEpsilon=(A+B*K)*initalnoise*(A+B*K)'+B*inputnoise*B';
        FullEpsilon=[FullEpsilon;NewEpsilon];
        error(k,:)=((A+B*K)*error(k-1,:)'+B*sqrt(inputnoise)*randn(2,1))';
        initalnoise=NewEpsilon;
    end
    FullEpsilon=[ZeroEpsilon;FullEpsilon];
    errorhat=[error(:,1),error(:,3)];
    for m=0:N
        varx(m+1,1)=FullEpsilon(1+4*m,1);
        vary(m+1,1)=FullEpsilon(3+4*m,3);
    end
    beta=0.6;% betas 0.3 and 0.88 , Noises Epsilon=[3 3 1.25 1.25] EpsilonTV=[3 1.25] z=[100 20 -4 0; -40 20 0 0] x=[40 -4 0 30]
    kappa=-2*log(1-beta);
    varxkappa=sqrt(kappa)*varx;
    varykappa=sqrt(kappa)*vary;
end

% function [varykappa,varxkappa]=errorestimate(N,T,K)
%     Epsilon=diag([0.25,0.25,0.028,0.028]);%0.25 0.25 0.028 0.028
%     ZeroEpsilon=Epsilon;
%     EpsilonTV=diag([0.44, 0.09]);% 0.44 0.09
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

function [arvector,brvector]=arandbr(varykappa,varxkappa,length,width,safefactor)
brvector=width*ones(size(varykappa))+safefactor*ones(size(varykappa))+varykappa;
arvector=length*ones(size(varxkappa))+safefactor*ones(size(varxkappa))+varxkappa;%4*safefactor
end

% function [xref]=computeReference(x0)
% if x0(2)>-6.5 && x0(2)<-2
%     xref=[0 -4 0 8];
%     elseif x0(2)>-2 && x0(2)<2
%         xref=[0 0 0 8];
%     else
%         xref=[0 4 0 8];
% end
% end

function [p0]=reducedstate(x0,N)
p0=[x0(1,1),x0(1,2)];
%make this into matrix
end

function p0=roundup(p0)
if p0(2)>2
        p0(2)=4;
    elseif  p0(2)<2 && p0(2)>-2
            p0(2)=0;
    else 
            p0(2)=-4;
   
end
end



function [zpred]=targetvehicleprediction(T,N,mpciter,z,zref,m,K) %z is the target vehicle state variable
A=[1 T 0 0;
0 1 0 0;
0 0 1 T;
0 0 0 1];
B=[0.5*T^2 0;  
T 0;
0 0.5*T^2;
0 T];
zpred=zeros(N+1,4,m);
    for k=1:m
        zpred(1,:,k)=z(mpciter+1,:,k);
        zinitialhat=zpred(1,:,k);
        for iter=2:N+1
            temp=A*zinitialhat'+B*K*(zinitialhat-zref(k,:))';
            zpred(iter,:,k)=temp';
            zinitialhat=temp';
        end
    end
end

function [xstop]=stoppingtime(x0,amin,N,T)
% xstop(1)=-x0(4)/(2*amin);
xstop=zeros(N+1,1);
    for r=0:N
        xstop(r+1)=-max(x0(4)^2-(amin*T*r)^2,0)/(2*amin);
    end
end

function [laneEV,laneTV,lanePED]=lanecheck(x0,zpred,m,mped,pedpred)
if x0(1,2)>-6.5 && x0(1,2)<-2 
    laneEV=-4;
    elseif x0(1,2)>-2 && x0(1,2)<2
        laneEV=0;
    else
        laneEV=4;    
end
laneTV=zeros(m,1);
for k=1:m
    if zpred(1,3,k)>-6.5 && zpred(1,3,k)<-2
        laneTV(k,1)=-4;
        elseif zpred(1,3,k)>-2 && zpred(1,3,k)<2
            laneTV(k,1)=0;
        else
            laneTV(k,1)=4;
    end
end
lanePED=zeros(m,1);
for k=1:mped
        if pedpred(1,3,k)>-3 && pedpred(1,3,k)<3
            lanePED(k,1)=0;
        else
            lanePED(k,1)=5;
        end
end
end

function [qx,qy,qt]=calculateQs(arvector,brvector,xstop,x0,m,safetywidth,laneEV,laneTV,N,zpred,arvectorped,mped,lanePED,pedpred)
if mped==0
qx=zeros(N+1,m);
qy=zeros(N+1,m);
qt=zeros(N+1,m);
    for k=1:m
        if x0(1)-zpred(1,1,k)<0 && laneEV==laneTV(k) && zpred(1,1,k)-x0(1)<30
        qx(:,k)=ones(N+1,1);
        qy(:,k)=zeros(N+1,1);
        qt(:,k)=-(-xstop-arvector+zpred(1,1,k));
        end
        if laneEV~=laneTV(k) && x0(1)-zpred(1,1,k)<0 && zpred(1,1,k)-x0(1)<30
        qx(:,k)=zeros(N+1,1);
        qy(:,k)=-ones(N+1,1);
        qt(:,k)=(brvector+safetywidth+zpred(1,3,k));
        end
    end
else
    qx=zeros(N+1,mped);
    qy=zeros(N+1,mped);
    qt=zeros(N+1,mped);
    for k=1:mped        
    if laneEV==lanePED && x0(1)-pedpred(1,1,k)<0
        qx(:,k)=ones(N+1,1);
        qy(:,k)=zeros(N+1,1);
        qt(:,k)=-(-xstop-arvectorped+pedpred(1,1,k));
    end
    end
end
end


function drawrectangles(x0,m,mped,zpred,pedpred,xstate,N,arvector,brvector,arvectorped,brvectorped,qx,qy,qt)
figure(1)
if m~=0
  axis([x0(1)-50 x0(1)+50  -10  10])
  cla
  plot(linspace(-100,1100,100),linspace(-2,-2,100),'--.k')
  hold on
  plot(linspace(-100,1100,100),linspace(2,2,100),'--.k')
  plot(linspace(-100,1100,100),linspace(6,6,100),'k','LineWidth',10)
  plot(linspace(-100,1100,100),linspace(-6,-6,100),'k','LineWidth',10)
else
  axis([x0(1)-30 x0(1)+30  -5  4])
  cla
  plot(linspace(-100,1100,100),linspace(-2,-2,100),'k','Linewidth',10)
  hold on
  plot(linspace(-100,1100,100),linspace(2,2,100),'k','LineWidth',10)
end
    
  
  

  
  
  EV=rectangle('Position', [x0(1)-2.5 x0(2)-1, 5  2],'EdgeColor', 'k', 'FaceColor','r');
  for i=2:N+1
  plot(xstate(i,1),xstate(i,2),'ko');
  end
       for q=1:m
            t = linspace(0,2*pi) ;
            x = zpred(1,1,q)+arvector(1)*cos(t);
            y = zpred(1,3,q)+brvector(1)*sin(t);
            plot(x,y,'r')
           
           TV(q)=rectangle('Position', [zpred(1,1,q)-2.5 zpred(1,3,q)-1, 5  2], 'EdgeColor', 'k', 'FaceColor','b');
           drawnow
       end
       for r=1:mped
           t = linspace(0,2*pi) ;
           x = pedpred(1,1,r)+arvectorped(1)*cos(t);
           y = pedpred(1,3,r)+brvectorped(1)*sin(t);
%            x1= pedpred(11,1,r)+arvectorped(11)*cos(t);
%            y1= pedpred(11,3,r)+brvectorped(11)*sin(t);
           plot(x,y,'r')%,x1,y1,'r')
          
%            plot(x1,y1,'r')
           for w=1:N+1
               plot(pedpred(w,1,r),pedpred(w,3,r),'kx')
           end
           Pedestrian(r)=rectangle('Position', [pedpred(1,1,r)-0.5 pedpred(1,3,r)-0.5, 1 1], 'EdgeColor', 'k', 'FaceColor','g');
           drawnow
       end
  if mped==0
%   for b=1:m 
%   for k=1:2 % add another for loop
%     f(k)=fimplicit(@(x,y) qx(k,b)*x+qy(k,b)*y+qt(k,b),[x0(1)-150 x0(1)+150 -10 10],'r');
%   end
%   end
  else
%   for b=1:mped 
%   for k=1:2 % add another for loop
%     f(k)=fimplicit(@(x,y) qx(k,b)*x+qy(k,b)*y+qt(k,b),[x0(1)-150 x0(1)+150 -10 10],'r');
%   end
%   end    
  end
  pause(0.7)

 

end

function drawrectanglesHL(p0,m,mped,zpredHL,pedpredHL,pstate,N,arvectorHL,brvectorHL,arvectorpedHL,brvectorpedHL)
figure(2)
if m~=0
axis([p0(1)-50 p0(1)+200  -10  10])
  cla
  plot(linspace(-100,1100,100),linspace(-2,-2,100),'--.k')
  hold on
  plot(linspace(-100,1100,100),linspace(2,2,100),'--.k')
  plot(linspace(-100,1100,100),linspace(6,6,100),'k','LineWidth',10)
  plot(linspace(-100,1100,100),linspace(-6,-6,100),'k','LineWidth',10)
else
  axis([p0(1)-50 p0(1)+100  -5  4])
  cla
  plot(linspace(-100,1100,100),linspace(-2,-2,100),'k','LineWidth',10)
  hold on
  plot(linspace(-100,1100,100),linspace(2,2,100),'k','LineWidth',10)
end
    
  

  
  
  
       for q=1:m
            t = linspace(0,2*pi) ;
            c = arvectorHL(2) ; d = brvectorHL(2) ;
            x = zpredHL(2,1,q)+c*cos(t) ;
            y = zpredHL(2,3,q)+d*sin(t) ;
            plot(x,y,'r')
            for w=1:2
%                  t = linspace(0,2*pi) ;
%                 a = 15 ; b = 3 ;
%                 x = zpredHL(w,1,q)+a*cos(t) ;
%                 y = zpredHL(w,3,q)+b*sin(t) ;
%                 plot(x,y,'r')
%                 TV(q)=rectangle('Position', [zpredHL(w,1,q)-2.5 zpredHL(w,3,q)-1, 5  2], 'EdgeColor', 'k', 'FaceColor','b');
                if mod(w,2)==0
                    TV(q)=rectangle('Position', [zpredHL(w,1,q)-2.5 zpredHL(w,3,q)-1, 5  2], 'EdgeColor', 'k', 'FaceColor','b');
                else
                    TV(q)=rectangle('Position', [zpredHL(w,1,q)-2.5 zpredHL(w,3,q)-1, 5  2], 'EdgeColor', 'k', 'FaceColor','y');
                end
                drawnow
            end
       end
            EV=rectangle('Position', [p0(1)-2.5 p0(2)-1, 5  2],'EdgeColor', 'k', 'FaceColor','r');
       for i=2:N+1
            plot(pstate(i,1),pstate(i,2),'ko');
       end
       for r=1:mped
           t = linspace(0,2*pi) ;
           x = pedpredHL(2,1,r)+arvectorpedHL(1)*cos(t);
           y = pedpredHL(2,3,r)+brvectorpedHL(1)*sin(t);
           plot(x,y,'r')
           for w=1:N+1
               plot(pedpredHL(w,1,r),pedpredHL(w,3,r),'kx')
           end
           Pedestrian(r)=rectangle('Position', [pedpredHL(1,1,r)-0.5 pedpredHL(1,3,r)-0.5, 1 1], 'EdgeColor', 'k', 'FaceColor','g');
           drawnow          
       end
 
  pause(0.7)

 


end
