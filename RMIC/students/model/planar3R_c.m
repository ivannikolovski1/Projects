% Calculate vector of centrifugal and coriolis load on the joints for
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
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [4x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% tauc [3x1]
%   joint torques required to compensate coriolis and centrifugal load

% 
% Quelle: IRT-Maple-Repo
% Datum: 2017-08-14 09:02
% Revision: bc23d638f29fba48702d62e294764e92d1da1df3
% (C) Institut für Regelungstechnik, Universität Hannover

function tauc = planar3R_c(q, dq, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
%% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
  'planar3R_coriolisvecJ_fixb_slag_vp1: q has to be [3x1] double');
assert(isa(dq,'double') && isreal(dq) && all(size(dq) == [3 1]), ...
  'planar3R_coriolisvecJ_fixb_slag_vp1: qD has to be [3x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
  'planar3R_coriolisvecJ_fixb_slag_vp1: pkin has to be [2x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [4 1]), ...
  'planar3R_coriolisvecJ_fixb_slag_vp1: m_num has to be [4x1] double'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [4,3]), ...
  'planar3R_coriolisvecJ_fixb_slag_vp1: rSges_num_mdh has to be [4x3] double');
assert(isa(Icges_num_mdh,'double') && isreal(Icges_num_mdh) && all(size(Icges_num_mdh) == [4 6]), ...
  'planar3R_coriolisvecJ_fixb_slag_vp1: Icges_num_mdh has to be [4x6] double'); 

%% Variable Initialization
qJ1s = q(1);
qJ2s = q(2);
qJ3s = q(3);

qJD1s = dq(1);
qJD2s = dq(2);
qJD3s = dq(3);

l1 = pkin(1);
l2 = pkin(2);

M0 = m_num(1);
M1 = m_num(2);
M2 = m_num(3);
M3 = m_num(4);

SX0 = rSges_num_mdh(1,1);
SY0 = rSges_num_mdh(1,2);
SZ0 = rSges_num_mdh(1,3);
SX1 = rSges_num_mdh(2,1);
SY1 = rSges_num_mdh(2,2);
SZ1 = rSges_num_mdh(2,3);
SX2 = rSges_num_mdh(3,1);
SY2 = rSges_num_mdh(3,2);
SZ2 = rSges_num_mdh(3,3);
SX3 = rSges_num_mdh(4,1);
SY3 = rSges_num_mdh(4,2);
SZ3 = rSges_num_mdh(4,3);

XXC0 = Icges_num_mdh(1,1);
XYC0 = Icges_num_mdh(1,4);
XZC0 = Icges_num_mdh(1,5);
YYC0 = Icges_num_mdh(1,2);
YZC0 = Icges_num_mdh(1,6);
ZZC0 = Icges_num_mdh(1,3);
XXC1 = Icges_num_mdh(2,1);
XYC1 = Icges_num_mdh(2,4);
XZC1 = Icges_num_mdh(2,5);
YYC1 = Icges_num_mdh(2,2);
YZC1 = Icges_num_mdh(2,6);
ZZC1 = Icges_num_mdh(2,3);
XXC2 = Icges_num_mdh(3,1);
XYC2 = Icges_num_mdh(3,4);
XZC2 = Icges_num_mdh(3,5);
YYC2 = Icges_num_mdh(3,2);
YZC2 = Icges_num_mdh(3,6);
ZZC2 = Icges_num_mdh(3,3);
XXC3 = Icges_num_mdh(4,1);
XYC3 = Icges_num_mdh(4,4);
XZC3 = Icges_num_mdh(4,5);
YYC3 = Icges_num_mdh(4,2);
YZC3 = Icges_num_mdh(4,6);
ZZC3 = Icges_num_mdh(4,3);

%% Symbolic Calculation
%From coriolisvec_joint_fixb_par1_matlab.m
t37 = qJD1s + qJD2s;
t38 = qJ1s + qJ2s;
t33 = sin(t38);
t34 = cos(t38);
t45 = t33 * SX2 + t34 * SY2;
t11 = t45 * t37;
t39 = sin(qJ1s);
t55 = l1 * qJD1s;
t51 = t39 * t55;
t9 = t11 + t51;
t64 = t37 ^ 2;
t63 = t33 * l2;
t62 = t34 * l2;
t61 = t39 * l1;
t40 = cos(qJ1s);
t60 = t40 * l1;
t35 = qJ3s + t38;
t29 = sin(t35);
t32 = qJD3s + t37;
t59 = t29 * t32;
t30 = cos(t35);
t58 = t30 * t32;
t57 = t33 * t37;
t56 = t34 * t37;
t7 = SX3 * t59 + SY3 * t58;
t54 = l2 * t57;
t24 = l2 * t56;
t41 = qJD1s ^ 2;
t53 = t41 * t60;
t52 = t54 + t7;
t50 = t40 * t55;
t16 = t34 * SX2 - t33 * SY2;
t14 = t30 * SX3 - t29 * SY3;
t13 = -t29 * SX3 - t30 * SY3;
t49 = -t32 * t13 + t54;
t48 = -t32 * t14 - t24;
t12 = -SX2 * t56 + SY2 * t57;
t8 = -SX3 * t58 + SY3 * t59;
t44 = -t14 - t62;
t10 = -t37 * t16 - t50;
t4 = t48 - t50;
t43 = t13 - t63;
t42 = t8 - t24;
t31 = t41 * t61;
t6 = t37 * t12 - t53;
t5 = t37 * t11 + t31;
t3 = t49 + t51;
t2 = t32 * t8 - t64 * t62 - t53;
t1 = t32 * t7 + t64 * t63 + t31;
t15 = [M3 * (t1 * (t44 - t60) + t4 * (t51 + t52) + t2 * (t43 - t61) - t3 * (t42 - t50)) + M2 * (t5 * (-t16 - t60) + t6 * (-t45 - t61) + (t10 - t12 + t50) * t9); (t1 * t44 + t2 * t43 + (-t49 + t52) * t4 + (-t42 + t48) * t3) * M3 + (t10 * t11 - t9 * t12 - t6 * t45 - t5 * t16 - (t10 * t45 + t16 * t9) * t37) * M2; (-t1 * t14 + t2 * t13 - t3 * t8 + t4 * t7 - (-t4 * t13 + t3 * t14) * t32) * M3;];
tauc  = t15 (:);
