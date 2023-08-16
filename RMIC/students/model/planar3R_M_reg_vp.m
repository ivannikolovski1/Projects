% Calculate minimal parameter regressor of joint inertia matrix for
% planar3R
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% pkin [2x1]
%   kinematic parameters
% 
% Output:
% MM_reg [((3+1)*3/2)x9]
%   minimal parameter regressor of joint inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% 
% Quelle: IRT-Maple-Repo
% Datum: 2017-08-14 09:02
% Revision: bc23d638f29fba48702d62e294764e92d1da1df3
% (C) Institut für Regelungstechnik, Universität Hannover

function MM_reg = planar3R_M_reg_vp(q, pkin)
%% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
  'planar3R_inertiaJ_regmin_slag_vp: q has to be [3x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
  'planar3R_inertiaJ_regmin_slag_vp: pkin has to be [2x1] double');

%% Variable Initialization
qJ1s = q(1);
qJ2s = q(2);
qJ3s = q(3);

l1 = pkin(1);
l2 = pkin(2);

%% Symbolic Calculation
%From inertia_joint_joint_fixb_regressor_minpar_matlab.m
t7 = sin(qJ3s);
t13 = t7 * l2;
t12 = sin(qJ2s) * l1;
t9 = cos(qJ3s);
t11 = t9 * t12;
t6 = cos(qJ2s) * l1;
t4 = t6 + l2;
t1 = -t7 * t12 + t9 * t4;
t5 = t9 * l2;
t2 = -t7 * t4 - t11;
t3 = [1 0 0 1 0.2e1 * t6 -0.2e1 * t12 1 0.2e1 * t1 0.2e1 * t2; 0 0 0 1 t6 -t12 1 t1 + t5 -t11 + (-l2 - t4) * t7; 0 0 0 1 0 0 1 0.2e1 * t5 -0.2e1 * t13; 0 0 0 0 0 0 1 t1 t2; 0 0 0 0 0 0 1 t5 -t13; 0 0 0 0 0 0 1 0 0;];
MM_reg  = t3 ;
