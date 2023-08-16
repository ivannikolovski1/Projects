% =========================================================================
% Author: Alexander Toedtheide, alexander.toedtheide@tum.de, 2015-02      =
% Editor: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

%% Get configuration parameters

if ~exist('constPar', 'var')
    error(['The configuration structure array constPar must be generated', ...
        'by the configuration file which will call this file.']);
else
    disp(['>> ' 'Parameters loaded']);
end

%%  Show joint angle constraints

disp(['>> ' 'Joint angle constraints'])
disp('     q_min      q_c0      q_max')
disp([constPar.q_min', constPar.q_bound', constPar.q_max'])

%% Load initial values for the optimizaiton vector

if isfield(constPar,'x0_in')
    constPar.x0 = constPar.x0_in;                        % inital vector
else
    constPar.x0 = ones(1,constPar.nopara);               % inital vector
end
display(constPar.x0, ['>> ' 'Initial values for the optimization vector'])

%save temp_params

%% Options for fmincon()

if ~isfield(constPar,'MaxIt')
    constPar.MaxIt = 1000;
end

options = optimset('Display','iter-detailed','MaxIter',constPar.MaxIt,'MaxFunEvals', constPar.maxiterFminCon,...
    'TolFun',1e-6,'UseParallel','always');

%% Start the opptimization

tic
RMIC_optim_problem(constPar.x0, constPar);
c = RMIC_nonlConstraints(constPar.x0,constPar);
if any(c > 1)
    warning('x0 violates the nonlinear constraints!');
end
T = table(constPar.n_cos,constPar.nop,constPar.tend,'VariableNames',{'n_cosines' 'n_samples' 't_end'});

disp('*******************************************************************')
disp('*                   TRAJECTORY OPTIMIZATION                       *')
disp('*******************************************************************')

display(T,['>> ' 'Trajectory settings'])
[optimOut.x_optim, optimOut.fitnessMin] = fmincon(@(x)RMIC_optim_problem(x,constPar),constPar.x0,[],[],[],[],[],[],@(x)RMIC_nonlConstraints(x,constPar),options);
toc

%% Display trajectory

close all
x = optimOut.x_optim;
constPar.n_cos = numel(x)/constPar.noj;
[q_opt,qD_opt,qDD_opt] = RMIC_calc_fourier_traj_poly(x, constPar.noj, constPar.nop, constPar.tend, constPar.omega, constPar.q_bound, constPar);
t_test                 = linspace(0,constPar.tend,constPar.nop);


% Trajectory animation

% preparing for loop until user either keypresses or clicks
global USER_RESPONDED
USER_RESPONDED = 0;

disp(['>> ' 'Starting trajectory animation'])
fig2 = figure(2);
    grid minor
    xlabel('x')
    ylabel('y')

% set(gcf,'WindowKeyPressFcn',@userRespondFcn,'WindowButtonDownFcn',...
%     @userRespondFcn,'DeleteFcn',@userRespondFcn)
set(fig2,'WindowKeyPressFcn',@userRespondFcn,'WindowButtonDownFcn',...
    @userRespondFcn,'DeleteFcn',@userRespondFcn)
axis xy equal, box on, hold on
axis([-2 2 -2 2])

r1 = 0.1;
r2 = 0.1;

q = q_opt';
t = t_test';

np   = 1000;          
tail = repmat([l1, 0, l1+l2, 0],[np,1]);

for i=1:100:size(q,2)
    cla
    % plotting tail
    alpha = linspace(0,1,size(tail,1)+1)';
    patch([tail(:,1);NaN],[tail(:,2);NaN],0,'EdgeColor','b','FaceColor',...
        'none','FaceVertexAlphaData',alpha,'EdgeAlpha','interp','LineWidth',1.5);
    patch([tail(:,3);NaN],[tail(:,4);NaN],0,'EdgeColor','r','FaceColor',...
        'none','FaceVertexAlphaData',alpha,'EdgeAlpha','interp','LineWidth',1.5);
    
    % plotting rods
    q1 = q(1,i);
    q2 = q(2,i);
    xm1 = double(subs(T_0_2(1,4)));
    ym1 = double(subs(T_0_2(2,4)));
    xm2 = double(subs(T_0_EE(1,4)));
    ym2 = double(subs(T_0_EE(2,4)));
 
    
    plot([0 xm1], [0 ym1],'b','LineWidth',6)
    plot([xm1 xm2], [ym1 ym2],'r','LineWidth',6)

    % plotting bobs
    p    = linspace(0,2*pi,17);
    sint = sin(p);
    cost = cos(p);
    patch(0 + r1*cost, 0 + r1*sint,0,'EdgeColor','k','FaceColor','k')
    patch(xm1 + r1*cost,ym1 + r1*sint,0,'EdgeColor','k','FaceColor','k')
    % patch(xm2+r2*cost,ym2+r2*sint,0,'EdgeColor','r','FaceColor','r')


    title(sprintf('time = %0.1f s',t(i)))
    drawnow
    tail = [tail(2:end,:);xm1,ym1,xm2,ym2];    
    pause(0.001)
    
    if(USER_RESPONDED == 1)
        USER_RESPONDED = 0;
        break;
    end
end
disp(['>> ' 'Finished trajectory animation'])
pause(3)


% Trajectory plots
disp(['>> ' 'Showing trajectory plots'])
close all
figure(1)
set(gcf, 'Position', get(0,'Screensize'))
subplot(3,2,1)
hold on
p1 = plot(t_test,rad2deg(q_opt(:,1)),'k','LineWidth',2);
p2 = plot(t_test,rad2deg(constPar.q_max(1))*ones(size(t_test)),'r--','LineWidth',2);
p3 = plot(t_test,rad2deg(constPar.q_min(1))*ones(size(t_test)),'b--','LineWidth',2);
leg11 = legend([p1 p2 p3],'$q_{1}$','$q_{1,max}$','$q_{1,min}$');
set(leg11,'Interpreter','latex','FontSize',20)
title('$q_{1}$ VS. Time','interpreter', 'latex','FontSize',20)
ylabel('$q_{1}(t)$ [deg]','interpreter', 'latex','FontSize',20)
grid minor  

subplot(3,2,2)
hold on
p4 = plot(t_test,rad2deg(q_opt(:,2)),'k','LineWidth',2);
p5 = plot(t_test,rad2deg(constPar.q_max(2))*ones(size(t_test)),'r--','LineWidth',2);
p6 = plot(t_test,rad2deg(constPar.q_min(2))*ones(size(t_test)),'b--','LineWidth',2);
leg12 = legend([p4 p5 p6],'$q_{2}$','$q_{2,max}$','$q_{2,min}$');
set(leg12,'Interpreter','latex','FontSize',20)
title('$q_{2}$ VS. Time','interpreter', 'latex','FontSize',20)
ylabel('$q_{2}(t)$ [deg]','interpreter', 'latex','FontSize',20)
grid minor

subplot(3,2,3)
hold on
p1 = plot(t_test,rad2deg(qD_opt(:,1)),'k','LineWidth',2);
p2 = plot(t_test,rad2deg(constPar.qd_max(1)*ones(size(q_opt(:,1)))),'r--','LineWidth',2);
p3 = plot(t_test,rad2deg(constPar.qd_min(1)*ones(size(q_opt(:,1)))),'b--','LineWidth',2);
leg11 = legend([p1 p2 p3],'$\dot{q}_{1}$','$\dot{q}_{1,max}$','$\dot{q}_{1,min}$');
set(leg11,'Interpreter','latex','FontSize',20)
title('$\dot{q}_{1}$ VS. Time','interpreter', 'latex','FontSize',20)
ylabel('$\dot{q}_{1}(t)$ [deg/s]','interpreter', 'latex','FontSize',20)
grid minor  

subplot(3,2,4)
hold on
p4 = plot(t_test,rad2deg(qD_opt(:,2)),'k','LineWidth',2);
p5 = plot(t_test,rad2deg(constPar.qd_max(2)*ones(size(q_opt(:,2)))),'r--','LineWidth',2);
p6 = plot(t_test,rad2deg(constPar.qd_min(2)*ones(size(q_opt(:,2)))),'b--','LineWidth',2);
leg12 = legend([p4 p5 p6],'$\dot{q}_{2}$','$\dot{q}_{2,max}$','$\dot{q}_{2,min}$');
set(leg12,'Interpreter','latex','FontSize',20)
title('$\dot{q}_{2}$ VS. Time','interpreter', 'latex','FontSize',20)
xlabel('Time [s]','interpreter', 'latex','FontSize',20)
ylabel('$\dot{q}_{2}(t)$ [deg/s]','interpreter', 'latex','FontSize',20)
grid minor

subplot(3,2,5)
hold on
p1 = plot(t_test,rad2deg(qDD_opt(:,1)),'k','LineWidth',2);
leg11 = legend([p1],'$\ddot{q}_{1}$');
set(leg11,'Interpreter','latex','FontSize',20)
title('$\ddot{q}_{1}$ VS. Time','interpreter', 'latex','FontSize',20)
xlabel('Time [s]','interpreter', 'latex','FontSize',20)
ylabel('$\ddot{q}_{1}(t)$ [deg/s]','interpreter', 'latex','FontSize',20)
grid minor  

subplot(3,2,6)
hold on
p4 = plot(t_test,rad2deg(qDD_opt(:,2)),'k','LineWidth',2);
leg12 = legend([p4],'$\ddot{q}_{2}$');
set(leg12,'Interpreter','latex','FontSize',20)
title('$\ddot{q}_{2}$ VS. Time','interpreter', 'latex','FontSize',20)
xlabel('Time [s]','interpreter', 'latex','FontSize',20)
ylabel('$\ddot{q}_{2}(t)$ [deg/s]','interpreter', 'latex','FontSize',20)
grid minor

