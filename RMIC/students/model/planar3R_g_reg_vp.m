% Calculate minimal parameter regressor of gravitation load for
% planar3R
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [3x1]
%   Generalized coordinates (joint angles) [rad]
% g_base [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [2x1]
%   kinematic parameters
% 
% Output:
% taug_reg [3x9]
%   minimal parameter regressor of gravitation joint torque vector

% 
% Quelle: IRT-Maple-Repo
% Datum: 2017-08-14 09:02
% Revision: bc23d638f29fba48702d62e294764e92d1da1df3
% (C) Institut für Regelungstechnik, Universität Hannover

function taug_reg = planar3R_g_reg_vp(q, g, ...
  pkin)
%% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
  'planar3R_gravloadJ_regmin_slag_vp: q has to be [3x1] double');
assert(isa(g,'double') && isreal(g) && all(size(g) == [3 1]), ...
  'planar3R_gravloadJ_regmin_slag_vp: g has to be [3x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
  'planar3R_gravloadJ_regmin_slag_vp: pkin has to be [2x1] double');

%% Variable Initialization
qJ1s = q(1);
qJ2s = q(2);
qJ3s = q(3);

g1 = g(1);
g2 = g(2);
g3 = g(3);

l1 = pkin(1);
l2 = pkin(2);

%% Symbolic Calculation
%From joint_gravload_fixb_regressor_minpar_matlab.m
t10 = qJ1s + qJ2s;
t12 = cos(qJ1s);
t11 = sin(qJ1s);
t9 = qJ3s + t10;
t8 = cos(t10);
t7 = sin(t10);
t6 = cos(t9);
t5 = sin(t9);
t4 = g1 * t8 + g3 * t7;
t3 = -g1 * t7 + g3 * t8;
t2 = g1 * t6 + g3 * t5;
t1 = -g1 * t5 + g3 * t6;
t13 = [0 g1 * t12 + g3 * t11 -g1 * t11 + g3 * t12 0 t4 t3 0 t2 t1; 0 0 0 0 t4 t3 0 t2 t1; 0 0 0 0 0 0 0 t2 t1;];
taug_reg  = t13 ;
