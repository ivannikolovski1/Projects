% =========================================================================
% Author: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

clc
close all

%% Initialization

T_smp      = 1E-3;
seed       = 0;
q_dist_var = (pi*0.005/2)^2 ;

%% Check in trajectory optimal parameters exist

if(~isfield(constPar,'a_opt'))
    warning('Trajectory parameters not available. Does <constPar.a_opt> have values?')
end

%% Open and configure model

sl_Modellname = 'RMIC_ident_experiment';
load_system(sl_Modellname)
open_system(sl_Modellname)
configSet = getActiveConfigSet(sl_Modellname);
set_param(configSet, 'Solver', 'ode4');
set_param(configSet, 'FixedStep', '1e-3');

%% << T3-A >> Run simulation
% The structure array 'sl' constains the required measurements, you can
% type the name of the variable in the command window to see its contents
% NOTE: remember to find a way to deal with the measurement noise

clc
constPar.test_trajectory = 0;

% <<YOUR CODE HERE>>
simOut      = sim(sl_Modellname, 'StopTime', '30');
sl          = get_simulink_outputs(simOut, sl_Modellname);

% Averages
q_avg = sl.q_sim/10;
dq_avg = sl.dq_sim/10;
ddq_avg = sl.ddq_sim/10;
tau_m_avg = sl.tau_m/10;
tau_frctn_avg = sl.tau_frctn/10;

% Constants in every simulation
q_ref = sl.q_ref;
dq_ref = sl.dq_ref;
ddq_ref = sl.ddq_ref;
t_ref = sl.t;

for repeat_avg = 1:9 
    simOut      = sim(sl_Modellname, 'StopTime', '30');
    sl          = get_simulink_outputs(simOut, sl_Modellname);
    
    q_avg = q_avg + sl.q_sim/10;
    dq_avg = dq_avg + sl.dq_sim/10;
    ddq_avg = ddq_avg + sl.ddq_sim/10;
    tau_m_avg = tau_m_avg + sl.tau_m/10;
    tau_frctn_avg = tau_frctn_avg + sl.tau_frctn/10;
end

%% << T3-B >> Identification
% Set the information matrix and information vector based on the
% measurements collected above

clc
% <<YOUR CODE HERE>>

aux_index = [-1,0];
for create_F = 1:length(q_ref)
    aux_index = aux_index + 2;
    q_ = [q_avg(create_F,1), q_avg(create_F,2)];
    qD_ = [dq_avg(create_F,1), dq_avg(create_F,2)];
    qDD_ = [ddq_avg(create_F,1), ddq_avg(create_F,2)];

    F_mat(aux_index,:) = [ qDD_(1), constPar.g*cos(q_(1)), -constPar.g*sin(q_(1)), qDD_(1) + qDD_(2), cos(q_(2))*(qDD_(1) + qDD_(2)) + constPar.g*cos(q_(1) + q_(2)) + qDD_(1)*cos(q_(2)) - qD_(1)*qD_(2)*sin(q_(2)) - qD_(2)*sin(q_(2))*(qD_(1) + qD_(2)), - sin(q_(2))*(qDD_(1) + qDD_(2)) - constPar.g*sin(q_(1) + q_(2)) - qDD_(1)*sin(q_(2)) - qD_(2)*cos(q_(2))*(qD_(1) + qD_(2)) - qD_(1)*qD_(2)*cos(q_(2));
                            0,         0,          0, qDD_(1) + qDD_(2),                         constPar.g*cos(q_(1) + q_(2)) + qDD_(1)*cos(q_(2)) - qD_(1)*qD_(2)*sin(q_(2)) + qD_(1)*sin(q_(2))*(qD_(1) + qD_(2)),                           qD_(1)*cos(q_(2))*(qD_(1) + qD_(2)) - qDD_(1)*sin(q_(2)) - constPar.g*sin(q_(1) + q_(2)) - qD_(1)*qD_(2)*cos(q_(2))];

    b_mat(aux_index,:) = [tau_m_avg(create_F,1);
                          tau_m_avg(create_F,2)];
    
end

beta_b_hat = inv(F_mat'*F_mat)*F_mat'*b_mat;


%% << T3-C >> Torque reproduction based on found minimal-parameters
% IMPORTANT: DO NOT change the code in this section!!!

clc 
figure
%%%%%%%%%%%%%% changed
points = size(sl.q_sim,1);
tau_m = tau_m_avg;
%%%%%%%%%%%%%%
tau_hat = reshape(F_mat*beta_b_hat,constPar.noj,points); 

close all

for i=1:constPar.noj
    subplot(2,1,i)
    hold on
    title('Joint torque VS. Time')
    plot(sl.t,tau_m(:,i),'b','LineWidth',3)
    plot(sl.t(1:100:end),tau_hat(i,1:100:end),'r--','LineWidth',3)
    ylabel(strcat('$\tau_',num2str(i),'(t)$ [Nm]'),'FontSize',20,'interpreter','latex')
    legend('Measured','Estimated')
    grid minor
end
xlabel('Time [s]','FontSize',20)

%% << T3-D >> Test with second trajectory

constPar.test_trajectory = 1;
lambdas = RMIC_get_fourier_lambdas(constPar.a_test, constPar.noj, constPar.nop, constPar.tend, constPar.omega, constPar.q_bound, constPar);
simOut  = sim(sl_Modellname, 'StopTime', '30'); 
sl      = get_simulink_outputs(simOut, sl_Modellname);
q       = sl.q_ref;
qD      = sl.dq_ref;
qDD     = sl.ddq_ref;
tau     = sl.tau_m;
points  = size(sl.q_sim,1);
%%%%% CHANGED
% in the ideal world, we would already have a 10 dimension beta
matrix_aux = RMIC_arm_inf_matrix(q, qD, qDD, points, constPar);
%%%%% CHANGED
tau_hat = reshape(matrix_aux(:,1:6)*beta_b_hat,constPar.noj,points); 

close all
figure
for i=1:constPar.noj
subplot(2,1,i)
hold on
title('Joint torque VS. Time')
plot(sl.t,tau(:,i),'b','LineWidth',3)
plot(sl.t(1:100:end),tau_hat(i,1:100:end),'r--','LineWidth',3)
ylabel(strcat('$\tau_',num2str(i),'(t)$ [Nm]'),'FontSize',20,'interpreter','latex')
legend('Measured','Estimated')
grid minor
end
xlabel('Time [s]','FontSize',20)