% =========================================================================
% Author: Alexander Toedtheide, alexander.toedtheide@tum.de, 2015-02      =
% Editor: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

function  [c,ceq] = RMIC_nonlConstraints(x,constPar)
% Nonlinear constraints function 
% c <= 0

q_min    = constPar.q_min;
q_max    = constPar.q_max;
qd_min   = constPar.qd_min;
qd_max   = constPar.qd_max;
noj      = constPar.noj;
nop      = constPar.nop;
tend     = constPar.tend;
omega    = constPar.omega;
q_bound  = constPar.q_bound;

%%  Trajectory calculation based on the 'x' parameter

[q,qD] = RMIC_calc_fourier_traj_poly(x, noj, nop, tend, omega, q_bound, constPar);

%%  Angle constraints

% Apply:
% * q                     [nop x 2]
% * qD                    [nop x 2]
% * q_max                 [1   x 2]
% * q_min                 [1   x 2]
% * qd_max                [1   x 2]
% * qd_min                [1   x 2]

c_ = NaN(4*noj,1);
for i = 1:noj
  c_(i)       =  max(q(:,i)) - q_max(i);
end

for i = 1:noj
  c_(noj+i)   =  -min(q(:,i)) + q_min(i);
end

for i = 1:noj
  c_(2*noj+i) =  max(qD(:,i)) - qd_max(i);
end

for i = 1:noj
  c_(3*noj+i) = -min(qD(:,i)) + qd_min(i);
end


c = c_;

ceq = [];


end