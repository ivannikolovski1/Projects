%% ************************************************************************
%                            Prepare workspace                            *
% *************************************************************************
clearvars
close all
clc

clear params

%% Robot parameters *******************************************************

params.g_base = [0,0,-9.81]'; % gravity vector
params.pkin   = [0.25,0.2,0.15]'; % length of robots links
params.m      = [100,4,2,1]'; % masses of robots links
params.r      = 0.05; % radius of robots links

% Center of mass of links
params.rSges  = [  0 , 0, 0;
                 0.15, 0, 0;
                 0.10, 0, 0;
                 0.05, 0, 0];

% Inertia tensor for links
params.Icges = [0.1,0.1,0.1,0,0,0;
    params.m(2)*params.r^2/2, params.m(2)*params.r^2/4 + params.m(2)*params.pkin(1)^2/12,params.m(2)*params.r^2/4 + params.m(2)*params.pkin(1)^2/12,0,0,0;
    params.m(3)*params.r^2/2, params.m(3)*params.r^2/4 + params.m(3)*params.pkin(2)^2/12,params.m(3)*params.r^2/4 + params.m(3)*params.pkin(2)^2/12,0,0,0;
    params.m(4)*params.r^2/2, params.m(4)*params.r^2/4 + params.m(4)*params.pkin(3)^2/12,params.m(4)*params.r^2/4 + params.m(4)*params.pkin(3)^2/12,0,0,0];

[params.mrSges, params.Ifges] = param_dep1(params.rSges, params.Icges, params.m);

params.q0  = [-pi/4;pi/3;-pi/2]; % initial joint angles
params.dq0 = [0;0;0]; % initial joint velocities

params.beta_b = planar3R_beta(params.pkin(1:2),params.m,params.mrSges,params.Ifges);

%% Simulation parameters **************************************************
params.sample_time = 1e-3; % length of one time step in the simulation %originall 1e-3
params.total_time  = 5.0; % total simulation time

%% Chosse the exercise ****************************************************
EXERCISE = 4;

%% Setting up exercises ***************************************************
%
% Variables:
% - flag_obs:
% switches the observer on or off, make sure to have implemented it
% - controller:
% selects the desired controller, 1 - pd controller, 2 - redundant
% impedance controller, 2 - full impedance controller
% - K_O:
% Filter constant for observer
% 

params.K_O = 100;
switch EXERCISE
    case 1
        params.controller   = 0;
        params.flag_obs     = false;
        params.flag_tau_ext = 0;
    case 2
        params.controller   = 0;
        params.flag_obs     = true;
        params.flag_tau_ext = 1;
    case 3
        params.controller   = 1;
        params.flag_obs     = false;
        params.flag_tau_ext = 0;
    case 4
        params.controller   = 2;
        params.flag_obs     = false;
        params.flag_tau_ext = 0;
    otherwise
        params.controller   = 2;
        params.flag_obs     = false;
        params.flag_tau_ext = 0;
end

%% Change this part for excercises II, II, and IV
params.flag_reg = true; 
% Uncomment the following line and fill it with the calculated base
% parameters
params.beta_b = [ 0.3008 1.3500 0 0.0679 0.4000 0 0.0050 0.0500 0]';

%% Plot robot

RUN_THIS = 0;
if RUN_THIS == 1
    figure;
    exc_plot(Time,Q',params,X_d');
end
