% Berechnung des Massenträgheitsmoments für verschobene Koordinaten
% 
% Eingabe:
% J_C
%   Massenträgheitsmoment um den Punkt C
% a
%   Vektor vom (neuen) Punkt P zum (alten) Punkt C
% m
%   Masse des Körpers
% 
% Ausgabe:
% J_P
%   Massenträgheitsmoment um den Punkt P

% Quelle:
% [1] http://de.wikipedia.org/wiki/Steinerscher_Satz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-07
% (c) Institut für Regelungstechnik, Universität Hannover

function J_P = param_dep3(J_C, a, m)

assert(all(size(J_C) == [3, 3]));
assert(all(size(a) == [3, 1]));
assert(all(size(m) == [1, 1]));

J_P = J_C + m * ...
    [a(2)^2+a(3)^2, -a(1)*a(2), -a(1)*a(3); ...
    -a(1)*a(2), a(1)^2+a(3)^2, -a(2)*a(3); ...
    -a(1)*a(3), -a(2)*a(3), a(1)^2+a(2)^2];