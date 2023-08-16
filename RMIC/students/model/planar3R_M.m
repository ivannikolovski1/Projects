% Calculate joint inertia matrix for
% planar3R
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% pkin [2x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [4x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% Mq [3x3]
%   inertia matrix

% 
% Quelle: IRT-Maple-Repo
% Datum: 2017-08-14 09:02
% Revision: bc23d638f29fba48702d62e294764e92d1da1df3
% (C) Institut für Regelungstechnik, Universität Hannover

function Mq = planar3R_M(q, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
%% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
  'planar3R_inertiaJ_slag_vp1: q has to be [3x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
  'planar3R_inertiaJ_slag_vp1: pkin has to be [2x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [4 1]), ...
  'planar3R_inertiaJ_slag_vp1: m_num has to be [4x1] double'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [4,3]), ...
  'planar3R_inertiaJ_slag_vp1: rSges_num_mdh has to be [4x3] double');
assert(isa(Icges_num_mdh,'double') && isreal(Icges_num_mdh) && all(size(Icges_num_mdh) == [4 6]), ...
  'planar3R_inertiaJ_slag_vp1: Icges_num_mdh has to be [4x6] double'); 

%% Variable Initialization
qJ1s = q(1);
qJ2s = q(2);
qJ3s = q(3);

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
%From inertia_joint_joint_floatb_twist_par1_matlab.m
t21 = sin(qJ1s);
t25 = t21 * l1;
t22 = cos(qJ1s);
t24 = t22 * l1;
t23 = ZZC2 + ZZC3;
t20 = qJ1s + qJ2s;
t17 = sin(t20);
t18 = cos(t20);
t10 = -t18 * SX2 + t17 * SY2;
t19 = qJ3s + t20;
t15 = sin(t19);
t16 = cos(t19);
t8 = -t16 * SX3 + t15 * SY3;
t9 = -t17 * SX2 - t18 * SY2;
t7 = -t15 * SX3 - t16 * SY3;
t4 = -t18 * l2 + t8;
t3 = -t17 * l2 + t7;
t12 = -t22 * SX1 + t21 * SY1;
t11 = -t21 * SX1 - t22 * SY1;
t6 = t10 - t24;
t5 = t9 - t25;
t2 = t4 - t24;
t1 = t3 - t25;
t13 = [ZZC1 + M1 * (t11 ^ 2 + t12 ^ 2) + M2 * (t5 ^ 2 + t6 ^ 2) + M3 * (t1 ^ 2 + t2 ^ 2) + t23; M2 * (t10 * t6 + t9 * t5) + M3 * (t3 * t1 + t4 * t2) + t23; M2 * (t10 ^ 2 + t9 ^ 2) + M3 * (t3 ^ 2 + t4 ^ 2) + t23; M3 * (t7 * t1 + t8 * t2) + ZZC3; M3 * (t7 * t3 + t8 * t4) + ZZC3; M3 * (t7 ^ 2 + t8 ^ 2) + ZZC3;];
%% Postprocessing: Reshape Output
% From vec2symmat_3_matlab.m
res = [t13(1) t13(2) t13(4); t13(2) t13(3) t13(5); t13(4) t13(5) t13(6);];
Mq  = res ;
