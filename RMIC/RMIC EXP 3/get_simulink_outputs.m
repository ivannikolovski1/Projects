% Extract all outputs from a simulink output structure
% 
% For this to work, configure your Simulink-Model the following way:
%  -> Simulation -> Model Configuration Parameters -> Data Import/Export
%  Format: "Structure with Time"
% Call your Simulink Modell with
%   simOut = sim(sl_Modellname, ...)
% 
% Input:
% simOut
%   SimOut struct created with the command
%   `simOut = sim(sl_Modellname, ...)`
%   Model has to be set to Output Format "Structure with Time".
% sl_Modellname
%   Name of the Simulink model. Has to be given without the file ending
%   (mdl, slx).
% 
% Output:
% sl
%   Structure with fields
%     't' time
%     fieldnames same as output block names in simulink model
%     assume all of the same time base

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut fuer Regelungstechnik, Universitaet Hannover

function [sl_out, sl_logsout] = get_simulink_outputs(simOut, sl_Modellname)

sl_out = struct('t', simOut.get('tout'));
sl_y = simOut.get('yout');

if isempty(sl_y)
  warning('No outputs in Simulink Model');
  return
end

% gehe alle Ausgänge durch und belege die Struktur
for j = 1:length(sl_y.signals)
  name = sl_y.signals(j).blockName(length(sl_Modellname)+2:end);
  try
    sl_out.(name) = sl_y.signals(j).values;
  end
end

% Gehe alle logsout-Signale durch und belege die Struktur
sl_logsout = struct('t', simOut.get('tout'));
sl_lo = simOut.get('logsout');
if ~isempty(sl_lo)
  Names = sl_lo.getElementNames;
  for j = 1:length(Names)
    name = Names{j};
    if isa(sl_lo.getElement(j).Values, 'struct')
      % Die Daten sind in einer Struktur enthalten. Kommt vor, wenn ein Bus
      % geloggt wird.
      % Alle Elemente der Struktur speichern
      % sl_logsout.(name) = struct('
      for fn = fieldnames(sl_lo.getElement(j).Values)'
        sl_logsout.(name).(fn{1}) = sl_lo.getElement(j).Values.(fn{1}).Data;
      end
    else
      sl_logsout.(name) = sl_lo.getElement(j).Values.Data;
    end
  end
end