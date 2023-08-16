% =========================================================================
% Author: Alexander Toedtheide, alexander.toedtheide@tum.de, 2015-02      =
% Editor: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

function [x] = RMIC_get_fourier_lambdas(param, noj, nop, tend, omega, q_bound, constPar)
% Calculation of Fourier series (based on Park2006)
% Arguments:
% * param [5*7x1]  -----> Parameters of the Fourier series
% * q     [nopx6]  -----> Joint angles
% * qD    [nopx6]  -----> Joint velocities
% * qDD   [nopx6]  -----> Joint accelerations
% * noj   [1]      -----> Number of joints
% * nop   [1]      -----> Number of discrete data points
% * omega [1]      -----> Angular frequency of one Fourier period
% * q_bound [1]    -----> Start/goal joint angle, for t_0 and t_end

%% Change of the parameters vector into a matrix structure

a_im = []; % Parameters from eq. (35)
for i = 1:noj
    a_im(i,:) = param(1+(i-1)*constPar.n_cos:constPar.n_cos+(i-1)*constPar.n_cos); 
end

%% Initialization

tf = 2*pi/omega;            
t  = linspace(0,tend,nop); % time points t

delta    = zeros(nop,noj);
D_delta  = zeros(nop,noj);
DD_delta = zeros(nop,noj);

% Iterate over time steps
for i = 1:nop
  % Iterate over joints
  for j = 1:noj
    % sum up
    delta_    = 0;
    D_delta_  = 0;
    DD_delta_ = 0;
    for mf = 1:constPar.n_cos
      % Eq. (35)
      delta_    = delta_ + a_im(j,mf)*cos((mf*pi/tf)*t(i));
      D_delta_  = D_delta_ - a_im(j,mf)*sin((mf*pi/tf)*t(i))*(mf*pi/tf);
      DD_delta_ = DD_delta_ - a_im(j,mf)*cos((mf*pi/tf)*t(i))*(mf*pi/tf)^2;
    end
    delta(i,j)    = delta_;
    D_delta(i,j)  = D_delta_;
    DD_delta(i,j) = DD_delta_;
    
  end
  
end

%% Calculation of the superimposed polynomial

x   = zeros(6,noj);
y   = zeros(nop,noj);
dy  = zeros(nop,noj);
ddy = zeros(nop,noj);

for j = 1:noj
  tp = [0 tend];
  
  % Definition of the boundary conditions
  % from eqs. (34) and (37)
  phi_edge = [q_bound(j)-delta(1,j) q_bound(j)-delta(end,j) -D_delta(1,j) -D_delta(end,j) -DD_delta(1,j) -DD_delta(end,j)]';
  
  % Matrix for the computation of the polynomial coefficients for the boundary conditions
  % from eq. (37)
  PolyMat = [1   tp(1)       tp(1)^2     tp(1)^3     tp(1)^4      tp(1)^5; ... % at 0
             1   tp(2)       tp(2)^2     tp(2)^3     tp(2)^4      tp(2)^5; ... % at tend
             0   1         2*tp(1)     3*tp(1)^2   4*tp(1)^3    5*tp(1)^4; ... % at 0
             0   1         2*tp(2)     3*tp(2)^2   4*tp(2)^3    5*tp(2)^4; ... % at tend
             0   0         2           6*tp(1)    12*tp(1)^2   20*tp(1)^3; ... % at 0
             0   0         2           6*tp(2)    12*tp(2)^2   20*tp(2)^3];    % at tend
  
  % Solve for the matrix of boundary conditions
  % form eq. (37)
  x(:,j) = PolyMat\phi_edge;
  
  % Computation of the discrete time points from the polynomial
  % from eq. (37)
%   y(:,j)   = polyval(flip(x(:,j)'),t)';
%   dy(:,j)  = polyval(polyder(flip(x(:,j)')),t)';
%   ddy(:,j) = polyval(polyder(polyder(flip(x(:,j)'))),t)';

end

% Correction of the Fourier series via the polynomial
% from eq. (34)
% q   =   y + delta;
% qD  =  dy + D_delta;
% qDD = ddy + DD_delta;
  
end