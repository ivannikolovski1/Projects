% Convert dynamic parameters for from parameter set 1 to set 2
% 
% set 1:
%   Mass, Center of Mass, Inertia around CoM
% set 2:
%   Mass, First Moment, Inertia round Frame Origin
% 
% Input:
% rSges_num_urdf [Nx3 double]
%   center of mass of all atlas arm bodies
% ISges_num [Nx6 double]
%   inertia of all atlas arm bodies around Center of Mass
%   order: xx, yy, zz, xy, xz, yz
% 
% Output:
% mrSges_num [Nx3 double]
%   first moment of all atlas arm bodies
% Ifges_num [Nx6 double]
%   inertia of all atlas arm bodies in around frame origin
%   order: xx, yy, zz, xy, xz, yz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover


function [mrSges_num, Ifges_num] = ...
  param_dep1(rSges_num, ISges_num, m_num)

N = size(rSges_num,1);
%% Erstes Moment
mrSges_num = NaN(N,3);
for i = 1:N
  mrSges_num(i,:) = m_num(i) * rSges_num(i,:);
end

%% Trägheitstensor um das Körperkoordinatensystem
Ifges_num = NaN(N,6);
for i = 1:N
  % Trägheitstensor um den Schwerpunkt
  I_iSi = param_dep2(ISges_num(i,:));
  
  % Steinerscher-Verschiebungssatz: Trägheitstensor um Koordinatensystem
  I_ii = param_dep3(I_iSi, rSges_num(i,:)', m_num(i));
  
  % Ausgabe zusammenstellen
  Ifges_num(i,:) = param_dep4(I_ii);
end
