function [e, V, exitflag, output, pstate] = solveOptimalControlProblemHL ...
            (runningcostsHL, terminalcostsHL, constraintsHL, ...
            terminalconstraintsHL, linearconstraintsHL, systemHL, N, t0, p0, e0, Tn, pref,eref, Qhl, Rhl, zpredHL,m,arvectorHL, brvectorHL,....
            pedpredHL,mped,arvectorpedHL,brvectorpedHL,eprevious,...
            atol_ode_sim, rtol_ode_sim, tol_opt, options, type)%eprevious
    p = zeros(N+1, length(p0));

    

    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    for i=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraintsHL(t0+i*Tn,p(i,:),e0(:,i));
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end

    % Solve optimization problem
    [e, V, exitflag, output] = fmincon(@(e) costfunctionHL(runningcostsHL, ...
        terminalcostsHL, systemHL, N, Tn, t0, p0, ...
        e, pref,eref, Qhl, Rhl, atol_ode_sim, rtol_ode_sim, type), e0, A, b, Aeq, beq, lb, ...
        ub, @(e) nonlinearconstraintsHL(constraintsHL, terminalconstraintsHL, ...
        systemHL, N, Tn, t0, p0, e, zpredHL, m,arvectorHL, brvectorHL,pedpredHL,mped,arvectorpedHL,brvectorpedHL,eprevious, ...
        atol_ode_sim, rtol_ode_sim, type), options);      
    pstate = computeOpenloopSolutionHL(systemHL, N, Tn, t0, p0, e, ...
                                atol_ode_sim, rtol_ode_sim, type);%problem was here instead of e I had e0
end

function costHL = costfunctionHL(runningcostsHL, terminalcostsHL, systemHL, ...
                    N, Tn, t0, p0, e, pref,eref, Qhl, Rhl,  ...
                    atol_ode_sim, rtol_ode_sim, type)
    costHL = 0;
    p = zeros(N+1, length(p0));
    p = computeOpenloopSolutionHL(systemHL, N, Tn, t0, p0, e , ...
                                atol_ode_sim, rtol_ode_sim, type);
    for i=1:N
        costHL = costHL+runningcostsHL(t0+i*Tn, p(i,:), e(:,i), pref,eref, Qhl, Rhl);
    end
    costHL = costHL+terminalcostsHL(t0+(N+1)*Tn, p(N+1,:),pref,eref, Qhl, Rhl);
end

function [c,ceq] = nonlinearconstraintsHL(constraintsHL, ...
    terminalconstraintsHL, systemHL, ...
    N, Tn, t0, p0, e,zpredHL, m ,arvectorHL, brvectorHL,pedpredHL,mped,arvectorpedHL,brvectorpedHL,eprevious,....
    atol_ode_sim, rtol_ode_sim, type)
    p = zeros(N+1, length(p0));
    p = computeOpenloopSolutionHL(systemHL, N, Tn, t0, p0, e, ...
                                atol_ode_sim, rtol_ode_sim, type);
    c = [];
    ceq = [];
    for i=1:N%change here
        [cnew, ceqnew] = constraintsHL(t0+i*Tn,p(i+1,:),e(:,i),zpredHL(i+1,:,:),m,arvectorHL(i+1), brvectorHL(i+1),pedpredHL(i+1,:,:),mped,arvectorpedHL(i+1),brvectorpedHL(i+1),eprevious);%,eprevious
        eprevious=e(:,i); % this constraint doesnt work for some reason
%         if any(e(1,1)~=eprevious(1))
%             keyboard()
%         end
        c = [c cnew];
        ceq = [ceq ceqnew];
    end

    [cnew, ceqnew] = terminalconstraintsHL(t0+(N+1)*Tn,p(N+1,:),zpredHL(N+1,:,:),m,arvectorHL(N+1,1), brvectorHL(N+1,1),pedpredHL(N+1,:,:),mped,arvectorpedHL(N+1,1),brvectorpedHL(N+1,1));
    c = [c cnew];
    ceq = [ceq ceqnew];

end


function p = computeOpenloopSolutionHL(systemHL, N, Tn, t0, p0, e,...
                                     atol_ode_sim, rtol_ode_sim, type)
    p(1,:) = p0;
    for i=1:N
        p(i+1,:) = dynamicHL(systemHL, Tn, t0, p(i,:), e(:,i),...
                             atol_ode_sim, rtol_ode_sim, type);
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