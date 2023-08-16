% Konvertiere einen Vektor in eine symmetrische Matrix
% 
% Eingabe:
% M_vec: [N*(N+1)/2 x 1]
%   Vektor mit unterem linken Teil einer symmetrischen Matrix (Zeilenweise)
% N: [1x1]
%   Dimension (Zeile bzw. Spalte) der Matrix
% 
% Ausgabe:
% M_symmat [N x N]
%   Symmetrische Matrix

% Quelle:
% http://de.mathworks.com/matlabcentral/newsreader/view_thread/169455

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
% (c) Institut für Regelungstechnik, Universität Hannover

function M_symmat = vec2symmat(M_vec, N)
if nargin == 1
  N =floor(sqrt(2*length(M_vec)));
end
M_symmat = NaN(N,N);
for i = 1:N
  for j = 1:N
    if j > i
      % tausche Zeilen und Spaltenindex (oberer rechter Teil)
      k = j*(j-1)/2 + i;
    else
      k = i*(i-1)/2 + j;
    end
    M_symmat(i,j) = M_vec(k);
  end
end