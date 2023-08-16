% =========================================================================
% Author: Alexander Toedtheide, alexander.toedtheide@tum.de, 2015-02      =
% Editor: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

function  [optim_fct,q,qD,qDD] = RMIC_optim_problem(param,constPar)
    % This function is called from the optimizer (e.g. fmincon)
    % Arguments:
    % * param       [5x7]  -----> Optimal trajectory parameters from the last
    %                             iteration
    % *constPar    []     -----> Configuration structure array
    
    noj          = constPar.noj;           % Number of joints
    nop          = constPar.nop;           % Number of discrete trajectory points
    tend         = constPar.tend;          % Time duration of a period
    omega        = constPar.omega;         % Angular frequency of a period

    %% Start/goal joint angles for t_0 and t_end
    if isfield(constPar,'q_bound')
        q_bound = constPar.q_bound;
    else
        q_bound = zeros(1,noj);
    end

    %% Calculation of the Fourier polynomial based ond the parameter 'param'
    % Eqs. (34) - (37)
    [q,qD,qDD] = RMIC_calc_fourier_traj_poly(param, noj, nop, tend, omega, q_bound, constPar);

    %% Construction of the information matrix
    % Eqs. (29) and (33)

    F = RMIC_arm_inf_matrix(q, qD, qDD, constPar.nop, constPar);                               

    %% Calculate condition number
    % Eq. (30)
    optim_fct =  cond(F'*F);
end
