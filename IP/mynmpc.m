function [t, x, u, z] = mynmpc(runningcosts, terminalcosts, ...
              constraints, terminalconstraints, ...
              linearconstraints, system, ...
              mpciterations, N, T,  tmeasure, xmeasure, u0, ...% here add z definently
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

           Q=varargin{12};
           R=varargin{13};
           zinital=varargin{14};
     
    
    
    
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
    
 
    m=size(zinital,1);
    [varykappa,varxkappa]=errorestimate(N,T);
    [arvector,brvector]=arandbr(varykappa,varxkappa)
    [zref]=targetvehiclereference(zinital,m);
    [z]=targetvehicle(T,mpciterations,zref,zinital,m);
    warning off all
    t = [];
    x = [];
    u = [];
    xinital=xmeasure;
    
    
    % Start of the NMPC iteration
    mpciter = 0;
    while(mpciter < mpciterations)
        % Step (1) of the NMPC algorithm:
        %   Obtain new initial value
        [t0, x0] = measureInitialValue ( tmeasure, xmeasure );
        % Step (2) of the NMPC algorithm:
        [zpred]=targetvehicleprediction(T,N,mpciter,z,zref,m); % prediction term
        [deltav]=deltavelocity(x0,zpred);
        [ar,br]=safetyrectangle(deltav,arvector,brvector);
        [C1EV,C2EV,C3EV,C4EV,C1TV,C2TV,C3TV,C4TV]=calculateCorners(x0,zpred,ar,br,N,m);
        [laneEV,laneTV]=lanecheck(x0,zpred,m);
        [xref]=computeReference(x0);
        [qx,qy,qt]=calculateQs(x0,zpred,laneEV,laneTV,N,C1EV,C2EV,C3EV,C4EV,C1TV,C2TV,C3TV,C4TV,m);
        xstate=computeOpenloopSolution(system, N, T, t0, x0, u0, xinital,atol_ode_sim, rtol_ode_sim, type);
        drawrectangles(x0,m,zpred,qx,qy,qt,N,C3TV,ar,br,xstate);
        %   Solve the optimal control problem
       
       
        t_Start = tic;
        [u_new, ~, exitflag, output] = solveOptimalControlProblem ...
            (runningcosts, terminalcosts, constraints, ...
            terminalconstraints, linearconstraints, system, ...
            N, t0, x0, u0, T, xref, Q, R,qx,qy,qt, xmeasure,m,  ...
            atol_ode_sim, rtol_ode_sim, tol_opt, options, type);
        t_Elapsed = toc( t_Start );
        
        %   Print solution
        if ( iprint >= 1 )
            printSolution(system, printHeader, printClosedloopData, ...
                          plotTrajectories, mpciter, T, t0, x0, u_new, xmeasure, ...
                          atol_ode_sim, rtol_ode_sim, type, iprint, ...
                          exitflag, output, t_Elapsed);
        end
        %   Store closed loop data
        t = [ t; tmeasure ];
        x = [ x; xmeasure ];
        u = [ u, u_new(:,1)];% this is changed
        %   Prepare restart
        u0 = shiftHorizon(u_new);
        % Step (3) of the NMPC algorithm:
        %   Apply control to process
        [tmeasure, xmeasure] = applyControl(system, T, t0, x0, u_new, xmeasure, ...
            atol_ode_real, rtol_ode_real, type);
        mpciter = mpciter+1;
    end
%     roadmap(z,x,T,m)
end

function [t0, x0] = measureInitialValue ( tmeasure, xmeasure )
    t0 = tmeasure;
    x0 = xmeasure;
end

function [tapplied, xapplied] = applyControl(system, T, t0, x0, u, xmeasure, ...
                                atol_ode_real, rtol_ode_real, type)
    xapplied = dynamic(system, T, t0, x0, u(:,1), xmeasure, ...
                       atol_ode_real, rtol_ode_real, type);
    tapplied = t0+T;
end

function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end

function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, constraints, terminalconstraints, ...
    linearconstraints, system, N, t0, x0, u0, T, xref, Q, R, qx, qy, qt, xmeasure ,m, ...
    atol_ode_sim, rtol_ode_sim, tol_opt, options, type)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u0, xmeasure, ...
                                atol_ode_sim, rtol_ode_sim, type);

    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    for k=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+k*T,x(k,:),u0(:,k));
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
        u, xref, Q, R, xmeasure,  atol_ode_sim, rtol_ode_sim, type), u0, A, b, Aeq, beq, lb, ...
        ub, @(u) nonlinearconstraints(constraints, terminalconstraints, ...
        system, N, T, t0, x0, u,qx,qy,qt, xmeasure,m,  ...
        atol_ode_sim, rtol_ode_sim, type), options);
end

function cost = costfunction(runningcosts, terminalcosts, system, ...
                    N, T, t0, x0, u, xref, Q, R, xinital ,...
                    atol_ode_sim, rtol_ode_sim, type)
    cost = 0;
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, xinital,   ...
                                atol_ode_sim, rtol_ode_sim, type);
    for k=1:N
        cost = cost+runningcosts(t0+k*T, x(k,:), u(:,k), xref, Q, R );
    end
    cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:));
end

function [c,ceq] = nonlinearconstraints(constraints, ...
    terminalconstraints, system, ...
    N, T, t0, x0, u, qx,qy,qt,xmeasure ,m, atol_ode_sim, rtol_ode_sim, type)% qs are here
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, xmeasure,...
                                atol_ode_sim, rtol_ode_sim, type);
    c = [];
    ceq = [];
    for k=2:N % here change
        [cnew, ceqnew] = constraints(t0+k*T,x(k,:),u(:,k),qx(k,:),qy(k,:),qt(k,:),m);%qs are here
        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    [cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:),qx(N+1,:),qy(N+1,:),qt(N+1,:),m);% and here possibly
    c = [c cnew];
    ceq = [ceq ceqnew];
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u , xmeasure , ...
                                     atol_ode_sim, rtol_ode_sim, type)
    x(1,:) = x0;
    for k=1:N
        x(k+1,:) = dynamic(system, T, t0, x(k,:), u(:,k) , xmeasure, ...
                             atol_ode_sim, rtol_ode_sim, type);
    end
end

function [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
             x0, u , xmeasure, atol_ode, rtol_ode, type)
    if ( strcmp(type, 'difference equation') )
        x = system(t0, x0, u, T, xmeasure);
        x_intermediate = [x0; x];
        t_intermediate = [t0, t0+T];
    elseif ( strcmp(type, 'differential equation') )
        options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);
        [t_intermediate,x_intermediate] = ode45(system, ...
            [t0, t0+T], x0, options, u);
        x = x_intermediate(size(x_intermediate,1),:);
    end
end

function printSolution(system, printHeader, printClosedloopData, ...
             plotTrajectories, mpciter, T, t0, x0, u, xmeasure, ...
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
        plotTrajectories(@dynamic, system, T, t0, x0, u, xmeasure ,atol_ode, rtol_ode, type)
    end
end

function printHeaderDummy(varargin)
end

function printClosedloopDataDummy(varargin)
end

function plotTrajectoriesDummy(varargin)
end

function [zref]=targetvehiclereference(zinital,m)
zref=zeros(m,4);
for k=1:m
if zinital(k,3)>-6.5 && zinital(k,3)<-2
zref(k,:)=[0 25 -4 0];
    elseif zinital(k,3)>-2 && zinital(k,3)<2
        zref(k,:)=[0 25 0 0];
    else
        zref(k,:)=[0 25 4 0];
    end
end
end

function [z]=targetvehicle(T,mpciterations,zref,zinital,m) %z is the target vehicle state variable
A=[1 T 0 0;
0 1 0 0;
0 0 1 T;
0 0 0 1];
B=[0.5*T^2 0;  
T 0;
0 0.5*T^2;
0 T];
K12=-0.55;
K21=-0.63;
K22=-1.15;
K=[0 K12 0 0;
    0 0 K21 K22];
z=zeros(mpciterations,4,m);
for k=1:m
z(1,:,k)=zinital(k,:)+sqrt((diag([0.25,0.25,0.028,0.028]))*rand(4,1))';
zinitialhat=z(1,:,k);
for iter=2:mpciterations
    temp=A*zinitialhat'+B*K*(zinitialhat-zref(k,:))'+B*sqrt(diag([0.44,0.09]))*randn(2,1);
    z(iter,:,k)=temp';
    zinitialhat=temp';
end
end
end

function [zpred]=targetvehicleprediction(T,N,mpciter,z,zref,m) %z is the target vehicle state variable
A=[1 T 0 0;
0 1 0 0;
0 0 1 T;
0 0 0 1];
B=[0.5*T^2 0;  
T 0;
0 0.5*T^2;
0 T];
K12=-0.55;
K21=-0.63;
K22=-1.15;
K=[0 K12 0 0;
    0 0 K21 K22];
zpred=zeros(N,4,m);
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



function [deltav]=deltavelocity(x0,zpred)% does this need to be length N? probably
amin=-9;
deltav=-1/(2*amin)*max(0,x0(1,4)^2-zpred(1,2)^2);
end






function [varykappa,varxkappa]=errorestimate(N,T)
    Epsilon=diag([3,3,1.25,1.25]);%0.25 0.25 0.028 0.028
    ZeroEpsilon=Epsilon;
    EpsilonTV=diag([3,1.25]);% 0.44 0.09
    e(1,:)=(sqrt(ZeroEpsilon)*randn(4,1))';
    A=[1 T 0 0;
    0 1 0 0;
    0 0 1 T;
    0 0 0 1];
    B=[0.5*T^2 0;  
    T 0;
    0 0.5*T^2;
    0 T];
    K12=-0.55;
    K21=-0.63;
    K22=-1.15;
    K=[0 K12 0 0;
        0 0 K21 K22];
    FullEpsilon=[];
    for k=2:N+1 
        NewEpsilon=(A+B*K)*Epsilon*(A+B*K)'+B*EpsilonTV*B';
        FullEpsilon=[FullEpsilon;NewEpsilon];
        e(k,:)=((A+B*K)*e(k-1,:)'+B*sqrt(EpsilonTV)*randn(2,1))';
        Epsilon=NewEpsilon;
    end
    FullEpsilon=[ZeroEpsilon;FullEpsilon];
    ehat=[e(:,1),e(:,3)];
    for m=0:N
        varx(m+1,1)=FullEpsilon(1+4*m,1);
        vary(m+1,1)=FullEpsilon(3+4*m,3);
    end
    beta=0.3;% betas 0.3 and 0.88 , Noises Epsilon=[3 3 1.25 1.25] EpsilonTV=[3 1.25] z=[100 20 -4 0; -40 20 0 0] x=[40 -4 0 30]
    kappa=-2*log(1-beta);
    varxkappa=sqrt(kappa)*varx;
    varykappa=sqrt(kappa)*vary;
end


function [arvector,brvector]=arandbr(varykappa,varxkappa)
width=2;
length=5;
safefactor=0.5;
brvector=width*ones(size(varykappa))+safefactor*ones(size(varykappa))+varykappa;
arvector=length*ones(size(varxkappa))+safefactor*ones(size(varxkappa))+varxkappa;
end



function [ar,br]=safetyrectangle(deltav,arvector,brvector)
ar=arvector+deltav*ones(size(arvector));
br=brvector;
end



function [C1EV,C2EV,C3EV,C4EV,C1TV,C2TV,C3TV,C4TV]=calculateCorners(x0,zpred,ar,br,N,m)

    yaw=x0(3);
    gamma=atan(2/5);% 2 is width 5 is length
    delta=1/2*sqrt(25+4);
    C1EV=[x0(1)+delta*cos(yaw+gamma),x0(2)+delta*sin(yaw+gamma)];
    C2EV=[x0(1)-delta*cos(yaw+gamma),x0(2)+delta*sin(yaw+gamma)];
    C3EV=[x0(1)-delta*cos(yaw+gamma),x0(2)-delta*sin(yaw+gamma)];
    C4EV=[x0(1)+delta*cos(yaw+gamma),x0(2)-delta*sin(yaw+gamma)];
C1TV=zeros(N+1,2,m);
C2TV=zeros(N+1,2,m);
C3TV=zeros(N+1,2,m);
C4TV=zeros(N+1,2,m);
for k=1:m 
    for p=1:N+1 %todo change it for more vehicles (add another for loop)
    C1TV(p,:,k)=[zpred(p,1,k)+ar(p,1);zpred(p,3,k)+br(p,1)];
    C2TV(p,:,k)=[zpred(p,1,k)-ar(p,1);zpred(p,3,k)+br(p,1)];
    C3TV(p,:,k)=[zpred(p,1,k)-ar(p,1);zpred(p,3,k)-br(p,1)];
    C4TV(p,:,k)=[zpred(p,1,k)+ar(p,1);zpred(p,3,k)-br(p,1)];
    end
end

end


function [laneEV,laneTV]=lanecheck(x0,zpred,m)
if x0(1,2)>-6.5 && x0(1,2)<-2 
    laneEV=-4;
    elseif x0(1,2)>-2 && x0(1,2)<2
        laneEV=0;
    else
        laneEV=4;    
end
laneTV=zeros(m,1);
for k=1:m
if zpred(k,3)>-6.5 && zpred(k,3)<-2
    laneTV(k,1)=-4;
    elseif zpred(k,3)>-2 && zpred(k,3)<2
        laneTV(k,1)=0;
    else
        laneTV(k,1)=4;
    end
end
end

function [xref]=computeReference(x0)
if x0(2)>-6.5 && x0(2)<-2
    xref=[0 -4 0 30];
    elseif x0(2)>-2 && x0(2)<2
        xref=[0 0 0 30];
    else
        xref=[0 4 0 30];
end
end



function [qx,qy,qt]=calculateQs(x0,zpred,laneEV,laneTV,N,C1EV,C2EV,C3EV,C4EV,C1TV,C2TV,C3TV,C4TV,m)
rlar=200;
rclose=90;% use function for rlar or rclose whichever one is in the paper
rlim=25; % what is rlim
qx=zeros(N+1,m);
qy=zeros(N+1,m);
qt=zeros(N+1,m);% or any nonpositive number
for k=1:m
if abs(x0(1,1)-zpred(1,1,k))>=rlar %works with numbers and works in simulation(checked off)
    qx(:,k)=zeros(N+1,1);
    qy(:,k)=zeros(N+1,1);
    qt(:,k)=zeros(N+1,1);
end
if -(x0(1,1)-zpred(1,1,k))<rlar && -(x0(1,1)-zpred(1,1,k))>rclose %works with numbers and works in simulation(checked off)
    qx(:,k)=ones(N+1,1);
    qy(:,k)=zeros(N+1,1);
    qt(:,k)=-C2TV(:,1,k);
end
if (x0(1,1)-zpred(1,1,k))<rlar && (x0(1,1)-zpred(1,1,k))>rclose %works (checked off)
    qx(:,k)=-1*ones(N+1,1);
    qy(:,k)=zeros(N+1,1);
    qt(:,k)=C1TV(:,1,k);
end
if -(x0(1,1)-zpred(1,1,k))<=rclose && -(x0(1,1)-zpred(1,1,k))>=0 && laneEV==laneTV(k)% case D 
    qx(:,k)=max(zeros(N+1,1),(C4EV(1,2)*ones(size(N+1,1))-C2TV(:,2,k))./((C4EV(1,1)*ones(size(N+1,1))-C2TV(:,1,k))));
    qy(:,k)=-1*ones(N+1,1);
    qt(:,k)=C2TV(:,2,k)-qx(:,k).*C2TV(:,1,k);
end
if -(x0(1,1)-zpred(1,1,k))<rclose && laneEV+4==laneTV(k) && x0(1,4)-zpred(1,2,k)>0  && x0(1,1)+1+rlim-zpred(1,1,k)<=0 % case E
    qx(:,k)=max(zeros(N+1,1),(C4EV(1,2)*ones(size(N+1,1))-C2TV(:,2,k))./((C4EV(1,1)*ones(size(N+1,1))-C2TV(:,1,k))));
    qy(:,k)=-1*ones(N+1,1);
    qt(:,k)=C2TV(:,2,k)-qx(:,k).*C2TV(:,1,k);
end
% if -(x0(1,1)-zpred(1,1,k))<=rlar && -(x0(1,1)-zpred(1,1,k))>=rclose && laneEV+4==laneTV(k) && x0(1,4)-zpred(1,2,k)>0 && x0(1,1)+1+rlim-zpred(1,1,k)<=0 %case E2
%     qx(:,k)=ones(N+1,1);
%     qy(:,k)=zeros(N+1,1);
%     qt(:,k)=-C2TV(:,1,k);
% end    
if -(x0(1,1)-zpred(1,1,k))>=0 && -(x0(1,1)-zpred(1,1,k))<=rclose && x0(1,1)+1+rlim-zpred(1,1,k)>0 && laneEV+4==laneTV(k) %case E3
    qx(:,k)=zeros(N+1,1);
    qy(:,k)=ones(N+1,1);
    qt(:,k)=-C3TV(:,2,k);
end
if abs(x0(1,1)-zpred(1,1,k))<rclose && laneEV>laneTV(k) % works(checked off)
    qx(:,k)=zeros(N+1,1);
    qy(:,k)=-1*ones(N+1,1);
    qt(:,k)=C2TV(:,2,k);
end

if -(x0(1,1)-zpred(1,1,k))<=rclose &&  -(x0(1,1)-zpred(1,1,k))>=0 && laneEV+8<=laneTV(k) % works (checked off)
    qx(:,k)=zeros(N+1,1);
    qy(:,k)=ones(N+1,1);
    qt(:,k)=-C3TV(:,2,k);
end

if (x0(1,1)-zpred(1,1,k))<=rclose && (x0(1,1)-zpred(1,1,k))>=0 && laneEV<=laneTV(k) %works (checked off)
    qx(:,k)=zeros(N+1,1);
    qy(:,k)=ones(N+1,1);
    qt(:,k)=-C4TV(:,2,k);
end

if (x0(1,1)-zpred(1,1,k))<=rclose && (x0(1,1)-zpred(1,1,k))>=0 && laneEV==laneTV(k) %works (checked off) but doesnt make sense(it assumes the other car wont drive into our car)
    qx(:,k)=zeros(N+1,1);
    qy(:,k)=zeros(N+1,1);
    qt(:,k)=zeros(N+1,1);
end

end
end


 function roadmap(z,x,T,m)
 scenario = drivingScenario();
   roadcenters = [0 0; 1000 0];
   lspc=lanespec(3,'Width',[4 4 4]);
   road(scenario,roadcenters,'Lanes',lspc);
   v = vehicle(scenario,'ClassID',1,'Width',2,'Length',5);
  
   waypoints1 = [x(:,1),x(:,2)];
   
   
    speedEV= x(:,4);
%    yaw=x(:,3);
   trajectory(v,waypoints1);
   for p=1:m
   k(p)= vehicle(scenario,'ClassID',1,'Width',2,'Length',5);
   speedsTV(:,p)= sqrt(z(:,2,p).^2+z(:,4,p).^2);
   waypoints2(:,:,p) = [z(:,1,p),z(:,3,p)];
   trajectory(k(p),waypoints2(:,:,p));
   end
   chasePlot(v,'Centerline','on')
%    
%       plot(scenario,'RoadCenters','on','Waypoints','on');
      
     
bep = birdsEyePlot('XLim',[-100 100],'YLim',[-35 35]);
olPlotter = outlinePlotter(bep);
lblPlotter = laneBoundaryPlotter(bep,'Color','r','LineStyle','-');
lbrPlotter = laneBoundaryPlotter(bep,'Color','g','LineStyle','-');
rbsEdgePlotter = laneBoundaryPlotter(bep);
legend('off');
while advance(scenario)
    rbs = roadBoundaries(v);
    [position,yaw,length,width,originOffset,color] = targetOutlines(v);
    lb = laneBoundaries(v,'XDistance',-50:5:50,'LocationType','Center', ...
        'AllBoundaries',false);
    plotLaneBoundary(rbsEdgePlotter,rbs)
    plotLaneBoundary(lblPlotter,{lb(1).Coordinates})
    plotLaneBoundary(lbrPlotter,{lb(2).Coordinates})
    plotOutline(olPlotter,position,yaw,length,width, ...
        'OriginOffset',originOffset,'Color',color)
  
   
end
%  
%      hold on;   
% %    while advance(scenario)
% %       
% %     pause(0.5)
% %   
% %    end
 end
 
function drawrectangles(x0,m,zpred,qx,qy,qt,N,C3TV,ar,br,xstate)

figure(7)
axis([x0(1)-100 x0(1)+100  -10  10])
  cla
  plot(linspace(-100,1100,100),linspace(-2,-2,100),'--.k')
  hold on
  plot(linspace(-100,1100,100),linspace(2,2,100),'--.k')
  plot(linspace(-100,1100,100),linspace(6,6,100),'k','LineWidth',10)
  plot(linspace(-100,1100,100),linspace(-6,-6,100),'k','LineWidth',10)
  

  
  
  EV=rectangle('Position', [x0(1)-2.5 x0(2)-1, 5  2],'EdgeColor', 'k', 'FaceColor','r');
  for i=2:3
  predicted=rectangle('Position', [xstate(i,1)-2.5 xstate(i,2)-1, 5 2],'EdgeColor', 'r');
  end
       for p=1:m
           safety(p)=rectangle('Position',[C3TV(1,1,p) C3TV(1,2,p), 2*ar(1,1) 2*br(1,1)]);
           TV(p)=rectangle('Position', [zpred(1,1,p)-2.5 zpred(1,3,p)-1, 5  2], 'EdgeColor', 'k', 'FaceColor','b');
       drawnow
       end
  
  for q=1:m
  for k=2 % add another for loop
  f(k)=fimplicit(@(x,y) qx(k,q)*x+qy(k,q)*y+qt(k,q),[x0(1)-150 x0(1)+150 -10 10],'r');
  end
  end  
  pause(0.7)
  set(f(k),'Visible','off')

 

end
 
