% Return the minimum parameter vector for
% planar3R
% Use Code from Maple symbolic Code Generation
% 
% Input:
% pkin [2x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh, Ifges_num_mdh [4x1]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% MPV [9x1]
%   base parameter vector (minimal parameter vector)

% 
% Quelle: IRT-Maple-Repo
% Datum: 2017-08-14 09:02
% Revision: bc23d638f29fba48702d62e294764e92d1da1df3
% (C) Institut für Regelungstechnik, Universität Hannover

function MPV = planar3R_beta(pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)

%% Coder Information
%#codegen
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [2 1]), ...
  'planar3R_convert_par2_MPV_fixb: pkin has to be [2x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [4 1]), ...
  'planar3R_convert_par2_MPV_fixb: m_num has to be [4x1] double'); 
assert(isa(mrSges_num_mdh,'double') && isreal(mrSges_num_mdh) && all(size(mrSges_num_mdh) == [4,3]), ...
  'planar3R_convert_par2_MPV_fixb: mrSges_num_mdh has to be [4x3] double');
assert(isa(Ifges_num_mdh,'double') && isreal(Ifges_num_mdh) && all(size(Ifges_num_mdh) == [4 6]), ...
  'planar3R_convert_par2_MPV_fixb: Ifges_num_mdh has to be [4x6] double'); 
%% Variable Initialization

l1 = pkin(1);
l2 = pkin(2);

M0 = m_num(1);
M1 = m_num(2);
M2 = m_num(3);
M3 = m_num(4);

MX0 = mrSges_num_mdh(1,1);
MY0 = mrSges_num_mdh(1,2);
MZ0 = mrSges_num_mdh(1,3);
MX1 = mrSges_num_mdh(2,1);
MY1 = mrSges_num_mdh(2,2);
MZ1 = mrSges_num_mdh(2,3);
MX2 = mrSges_num_mdh(3,1);
MY2 = mrSges_num_mdh(3,2);
MZ2 = mrSges_num_mdh(3,3);
MX3 = mrSges_num_mdh(4,1);
MY3 = mrSges_num_mdh(4,2);
MZ3 = mrSges_num_mdh(4,3);

XX0 = Ifges_num_mdh(1,1);
XY0 = Ifges_num_mdh(1,4);
XZ0 = Ifges_num_mdh(1,5);
YY0 = Ifges_num_mdh(1,2);
YZ0 = Ifges_num_mdh(1,6);
ZZ0 = Ifges_num_mdh(1,3);
XX1 = Ifges_num_mdh(2,1);
XY1 = Ifges_num_mdh(2,4);
XZ1 = Ifges_num_mdh(2,5);
YY1 = Ifges_num_mdh(2,2);
YZ1 = Ifges_num_mdh(2,6);
ZZ1 = Ifges_num_mdh(2,3);
XX2 = Ifges_num_mdh(3,1);
XY2 = Ifges_num_mdh(3,4);
XZ2 = Ifges_num_mdh(3,5);
YY2 = Ifges_num_mdh(3,2);
YZ2 = Ifges_num_mdh(3,6);
ZZ2 = Ifges_num_mdh(3,3);
XX3 = Ifges_num_mdh(4,1);
XY3 = Ifges_num_mdh(4,4);
XZ3 = Ifges_num_mdh(4,5);
YY3 = Ifges_num_mdh(4,2);
YZ3 = Ifges_num_mdh(4,6);
ZZ3 = Ifges_num_mdh(4,3);

%% Symbolic Calculation
%From minimal_parameter_vector_fixb_matlab.m
t24 = M2 + M3;
t1 = [l1 ^ 2 * t24 + ZZ1; l1 * t24 + MX1; MY1; l2 ^ 2 * M3 + ZZ2; M3 * l2 + MX2; MY2; ZZ3; MX3; MY3;];
MPV  = t1 ;
