% Calculate minimal parameter regressor of coriolis matrix for
% planar3R
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% qD [3x1]
%   Generalized velocities (joint velocities) [rad/s]
% pkin [2x1]
%   kinematic parameters
% 
% Output:
% cmat_reg [(3*%NQJ)%x9]
%   minimal parameter regressor of coriolis matrix

% 
% Quelle: IRT-Maple-Repo
% Datum: 2017-08-14 09:02
% Revision: bc23d638f29fba48702d62e294764e92d1da1df3
% (C) Institut für Regelungstechnik, Universität Hannover

function cmat_reg = planar3R_C_reg_vp(q, qD, ...
  pkin)
%% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
  'planar3R_coriolismatJ_fixb_regmin_slag_vp: q has to be [3x1] double');
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [3 1]), ...
  'planar3R_coriolismatJ_fixb_regmin_slag_vp: qD has to be [3x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
  'planar3R_coriolismatJ_fixb_regmin_slag_vp: pkin has to be [2x1] double');

%% Variable Initialization
qJ1s = q(1);
qJ2s = q(2);
qJ3s = q(3);

qJD1s = qD(1);
qJD2s = qD(2);
qJD3s = qD(3);

l1 = pkin(1);
l2 = pkin(2);

%% Symbolic Calculation
%From coriolismat_joint_fixb_regressor_minpar_matlab.m
t44 = -l2 / 0.2e1;
t24 = cos(qJ2s);
t43 = t24 * l1;
t18 = l2 + t43;
t21 = sin(qJ3s);
t42 = t21 * t18;
t41 = t21 * t24;
t23 = cos(qJ3s);
t40 = t23 * t18;
t22 = sin(qJ2s);
t39 = t23 * t22;
t38 = l1 * qJD1s;
t37 = l1 * qJD2s;
t36 = l2 * qJD2s;
t35 = l2 * qJD3s;
t28 = -t43 / 0.2e1;
t25 = t28 + l2 / 0.2e1 + t18 / 0.2e1;
t3 = t25 * t21;
t34 = t3 * qJD1s;
t4 = t25 * t23;
t33 = t4 * qJD1s;
t17 = t21 * t22 * l1;
t7 = t17 - t40;
t32 = t7 * qJD1s;
t8 = l1 * t39 + t42;
t31 = t8 * qJD1s;
t11 = (t39 + t41) * l1;
t30 = t11 * qJD1s;
t12 = t23 * t43 - t17;
t29 = t12 * qJD1s;
t27 = l1 * (-qJD1s - qJD2s);
t26 = l2 * (-qJD2s - qJD3s);
t10 = t12 * qJD2s;
t9 = t11 * qJD2s;
t6 = t8 * qJD3s;
t5 = t7 * qJD3s;
t2 = t17 - t40 / 0.2e1 + (t44 + t28) * t23;
t1 = t21 * t44 - t42 / 0.2e1 + (-t39 - t41 / 0.2e1) * l1;
t13 = [0 0 0 0 -t22 * t37 -t24 * t37 0 -t9 - t6 -t10 + t5; 0 0 0 0 t22 * t27 t24 * t27 0 t1 * qJD3s - t30 - t9 t2 * qJD3s - t10 - t29; 0 0 0 0 0 0 0 t1 * qJD2s - t31 - t6 t2 * qJD2s + t32 + t5; 0 0 0 0 t22 * t38 t24 * t38 0 -t3 * qJD3s + t30 -t4 * qJD3s + t29; 0 0 0 0 0 0 0 -t21 * t35 -t23 * t35; 0 0 0 0 0 0 0 t21 * t26 - t34 t23 * t26 - t33; 0 0 0 0 0 0 0 t3 * qJD2s + t31 t4 * qJD2s - t32; 0 0 0 0 0 0 0 t21 * t36 + t34 t23 * t36 + t33; 0 0 0 0 0 0 0 0 0;];
cmat_reg  = t13 ;
