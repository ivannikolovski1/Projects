% Calculate Gravitation load on the joints for
% planar3R
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [2x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh [4x1]
%   dynamic parameters
% 
% Output:
% taug [3x1]
%   joint torques required to compensate gravitation load

% 
% Quelle: IRT-Maple-Repo
% Datum: 2017-08-14 09:02
% Revision: bc23d638f29fba48702d62e294764e92d1da1df3
% (C) Institut für Regelungstechnik, Universität Hannover

function taug = planar3R_g(q, g, ...
  pkin, m_num, rSges_num_mdh)
%% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
  'planar3R_gravloadJ_floatb_twist_slag_vp1: q has to be [3x1] double');
assert(isa(g,'double') && isreal(g) && all(size(g) == [3 1]), ...
  'planar3R_gravloadJ_floatb_twist_slag_vp1: g has to be [3x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
  'planar3R_gravloadJ_floatb_twist_slag_vp1: pkin has to be [2x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [4 1]), ...
  'planar3R_gravloadJ_floatb_twist_slag_vp1: m_num has to be [4x1] double'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [4,3]), ...
  'planar3R_gravloadJ_floatb_twist_slag_vp1: rSges_num_mdh has to be [4x3] double');

%% Variable Initialization
qJ1s = q(1);
qJ2s = q(2);
qJ3s = q(3);

g1 = g(1);
g2 = g(2);
g3 = g(3);

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

%% Symbolic Calculation
%From joint_gravload_floatb_twist_par1_matlab.m
t9 = sin(qJ1s);
t18 = t9 * l1;
t10 = cos(qJ1s);
t17 = t10 * l1;
t8 = qJ1s + qJ2s;
t5 = sin(t8);
t6 = cos(t8);
t16 = -t6 * SX2 + t5 * SY2;
t7 = qJ3s + t8;
t3 = sin(t7);
t4 = cos(t7);
t15 = -t4 * SX3 + t3 * SY3;
t14 = -t5 * SX2 - t6 * SY2;
t13 = -t3 * SX3 - t4 * SY3;
t12 = -t6 * l2 + t15;
t11 = -t5 * l2 + t13;
t1 = [-M1 * (g1 * (-t10 * SX1 + t9 * SY1) + g3 * (-t9 * SX1 - t10 * SY1)) - M2 * (g1 * (t16 - t17) + g3 * (t14 - t18)) - M3 * (g1 * (t12 - t17) + g3 * (t11 - t18)) -M2 * (g1 * t16 + g3 * t14) - M3 * (g1 * t12 + g3 * t11) -M3 * (g1 * t15 + g3 * t13)];
taug  = t1 (:);
