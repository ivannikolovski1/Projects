% =========================================================================
% Author: Alexander Toedtheide, alexander.toedtheide@tum.de, 2015-02      =
% Author: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

close all

%% Assign values to variables

m1    = 10;
m2    = 10;
l1    = 1; 
lc1   = l1/2 ;
l2    = 1;
lc2   = l2/2;
h1    = 0.1;
delta = 0.05;
g     = 9.81;

I_com1 = diag([1/12*m1*(h1^2 + h1^2), 1/12*m1*(l1^2 + h1^2), 1/12*m1*(l1^2 + h1^2)]); 
I_com2 = diag([1/12*m2*(h1^2 + h1^2), 1/12*m2*(l2^2 + h1^2), 1/12*m2*(l2^2 + h1^2)]); 

J1 = I_com1 - m1*skew([lc1 0 0])*skew([lc1 0 0]);
J2 = I_com2 - m2*skew([lc2 0 0])*skew([lc2 0 0]);

[I1xx, I1xy, I1xz, I1yy, I1yz, I1zz] = deal(I_com1(1,1),I_com1(1,2),I_com1(1,3),I_com1(2,2),I_com1(2,3),I_com1(3,3));
[I2xx, I2xy, I2xz, I2yy, I2yz, I2zz] = deal(I_com2(1,1),I_com2(1,2),I_com2(1,3),I_com2(2,2),I_com2(2,3),I_com2(3,3));

[XX1, XY1, XZ1, YY1, YZ1, ZZ1] = deal(J1(1,1),J1(1,2),J1(1,3),J1(2,2),J1(2,3),J1(3,3));
[XX2, XY2, XZ2, YY2, YZ2, ZZ2] = deal(J2(1,1),J2(1,2),J2(1,3),J2(2,2),J2(2,3),J2(3,3));

mX1 = m1*lc1;
mY1 = 0;
mZ1 = 0;
mX2 = m2*lc2;
mY2 = 0;
mZ2 = 0;
 
% MDH parameters
a1     = 0;
a2     = l1;
alpha1 = 0;
alpha2 = 0;
d1     = 0;
d2     = delta;

% Friction coefficients
f_nu =  2;
f_c  = 1;

%% Construction of the standard inertial parameters; i.e. w.r.t. the links's frames

theta = [XX1, XY1, XZ1, YY1, YZ1, ZZ1, mX1, mY1, mZ1, m1 ...
         XX2, XY2, XZ2, YY2, YZ2, ZZ2, mX2, mY2, mZ2, m2];

constPar.theta       = theta;
constPar.kinParamL1  = [a1 d1 alpha1];
constPar.kinParamL2  = [a2 d2 alpha2];

%% Additional parameters

constPar.g           = 9.81;
constPar.useCoulomb  = 0;
constPar.tanh_coef   = 1000; % used for the Coulomb friction, this is used to model the discontinuity

%% Definition of the joint constraints

constPar.q_min     = deg2rad([-360 -360]);    % min joint angles
constPar.q_max     = deg2rad([360 360]);      % max joint angles
constPar.qd_min    = [-2*pi -2*2*pi];         % as per the manufacturer data sheet        
constPar.qd_max    = [2*pi 2*2*pi];           % as per the manufacturer data sheet

%% Maximum number of iterations

constPar.MaxIt     = 1000;

%% Optimization settings for the trajectory

rng(0,'twister');

constPar.q_bound          = deg2rad([0 0]);                  % boundary conditions for the trajectory optimization
constPar.noj              = 2;                               % number of joints
constPar.tend             = 30;                              % basic time interval of fourier series
constPar.omega            = 2*pi*(1/constPar.tend);          % basic frequency
constPar.nop              = 30000;                           % number of samples in a trajectory
constPar.considerFrct     = 1;                               % 1 = consider friction within information matrix
constPar.maxiterFminCon   = 1e6;                             % maximum number of iteration of fminCon()
constPar.n_cos            = 5;                               % this is the number of cosine functions included in the trajectory

% Generate random initial amplitudes within joint limits
a  = constPar.q_min(1);
b  = constPar.q_max(1);
r1 = (b-a).*rand(constPar.n_cos,1) + a;
a  = constPar.q_min(2);
b  = constPar.q_max(2);
r2 = (b-a).*rand(constPar.n_cos,1) + a;

constPar.x0_in            = [r1;r2];                         % Initial values for the optimization variables
constPar.regIndizes       = [2 6];                           % Dimensions of the reduced regressor matrix 

%% Test trajectory

constPar.test_trajectory = 0;
constPar.a_test          = [3.85801193867399;7.94780219896543;10.2167909900805;1.68259554848223;5.39736738809550;-5.16773282000065;24.8971295748132;-5.18304286269404;4.46639184215353;-4.23314811858320];
disp(['>> ' 'constPar structure created'])

%% Start the script

run RMIC_call_optim_traj
constPar.a_opt  = optimOut.x_optim;

disp(['>> ' 'Trajectory parameters stored'])

%% Polynomial coefficients for the found trajectory

lambdas = RMIC_get_fourier_lambdas(constPar.a_opt, constPar.noj, constPar.nop, constPar.tend, constPar.omega, constPar.q_bound, constPar);
disp(['>> ' 'Fourier polynomial defined'])