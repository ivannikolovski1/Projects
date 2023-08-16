% *************************************************************************
% Calculate minimal parameter regressor of coriolis joint torque vector for
% planar3R
% *************************************************************************
% 
% Input:
% q [3x1]
%   Joint angles [rad]
% qD [3x1]
%   Joint velocities [rad/s]
% qD [3x1]
%   Joint accelerations [rad/s]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [2x1]
%   kinematic parameters
% 
% Output:
% tau_reg [3x9]
%   minimal parameter regressor of inverse dynamics joint torque vector
%
% 
% (C) Lehrstuhl für Robotik und Systemintelligenz, TUM

function tau_reg = planar3R_Y(q, dq, ddq, g, ...
  pkin)
%% Coder Information
    %#codegen
    assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
      'planar3R_invdynJ_fixb_regmin_slag_vp: q has to be [3x1] double');
    assert(isa(dq,'double') && isreal(dq) && all(size(dq) == [3 1]), ...
      'planar3R_invdynJ_fixb_regmin_slag_vp: qD has to be [3x1] double');
    assert(isa(ddq,'double') && isreal(ddq) && all(size(ddq) == [3 1]), ...
      'planar3R_invdynJ_fixb_regmin_slag_vp: qDD has to be [3x1] double');
    assert(isa(g,'double') && isreal(g) && all(size(g) == [3 1]), ...
      'planar3R_invdynJ_fixb_regmin_slag_vp: g has to be [3x1] double');
    assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
      'planar3R_invdynJ_fixb_regmin_slag_vp: pkin has to be [2x1] double');

%% Variable Initialization
    qJ1s = q(1);
    qJ2s = q(2);
    qJ3s = q(3);

    qJD1s = dq(1);
    qJD2s = dq(2);
    qJD3s = dq(3);

    qJDD1s = ddq(1);
    qJDD2s = ddq(2);
    qJDD3s = ddq(3);

    g1 = g(1);
    g2 = g(2);
    g3 = g(3);

    l1 = pkin(1);
    l2 = pkin(2);

%% Symbolic Calculation
    %From invdyn_joint_fixb_regressor_minpar_matlab.m
    t27 = cos(qJ2s);
    t52 = t27 * l1;
    t24 = sin(qJ2s);
    t26 = cos(qJ3s);
    t51 = t24 * t26;
    t20 = qJDD1s + qJDD2s;
    t15 = qJDD3s + t20;
    t50 = t26 * t15;
    t49 = l1 * qJD1s;
    t48 = qJD2s * t27;
    t23 = sin(qJ3s);
    t47 = qJD3s * t23;
    t46 = qJD3s * t26;
    t22 = qJ1s + qJ2s;
    t41 = -qJD2s - qJD3s;
    t16 = qJD1s - t41;
    t45 = -qJD3s + t16;
    t44 = qJDD1s * t24;
    t43 = -t15 - qJDD1s;
    t19 = qJ3s + t22;
    t11 = sin(t19);
    t12 = cos(t19);
    t14 = qJDD1s * t52;
    t39 = t24 * t49;
    t2 = t20 * l2 - qJD2s * t39 + t14;
    t42 = g1 * t12 + g3 * t11 + t26 * t2;
    t17 = sin(t22);
    t18 = cos(t22);
    t40 = g1 * t18 + g3 * t17 + t14;
    t38 = t23 * t48;
    t37 = -g1 * t17 + g3 * t18;
    t36 = t23 * t44;
    t21 = qJD1s + qJD2s;
    t3 = t21 * l2 + t27 * t49;
    t35 = t45 * t3;
    t34 = -g1 * t11 + g3 * t12 + t39 * t47;
    t33 = qJD1s * (-qJD2s + t21);
    t32 = qJD2s * (-qJD1s - t21);
    t31 = t41 * t16;
    t30 = (-qJD1s - t16) * t48;
    t13 = l2 + t52;
    t29 = qJD3s * (-t13 * t16 - t3);
    t28 = cos(qJ1s);
    t25 = sin(qJ1s);
    t1 = [qJDD1s g1 * t28 + g3 * t25 -g1 * t25 + g3 * t28 t20 (t20 * t27 + t24 * t32) * l1 + t40 ((-qJDD1s - t20) * t24 + t27 * t32) * l1 + t37 t15 t13 * t50 + t23 * t29 + (t23 * t30 + (t43 * t23 + (-qJD1s * qJD3s + t31) * t26) * t24) * l1 + t42 (-t13 * t15 - t2) * t23 + t26 * t29 + (t26 * t30 + (-t23 * t31 + t43 * t26) * t24) * l1 + t34; 0 0 0 t20 t24 * l1 * t33 + t40 (t27 * t33 - t44) * l1 + t37 t15 -t3 * t47 + (-t16 * t47 + t50) * l2 + (-t36 + (-t24 * t46 - t38 - (-t23 * t27 - t51) * t16) * qJD1s) * l1 + t42 -t3 * t46 - t23 * t2 + (-t23 * t15 - t16 * t46) * l2 + (-t26 * t44 + (-t26 * t48 + (-t23 * t24 + t26 * t27) * t16) * qJD1s) * l1 + t34; 0 0 0 0 0 0 t15 t23 * t35 + (-t36 + (t45 * t51 - t38) * qJD1s) * l1 + t42 (-t16 * t39 - t2) * t23 + (t35 + (-qJD1s * t48 - t44) * l1) * t26 + t34;];
    tau_reg  = t1 ;
end
