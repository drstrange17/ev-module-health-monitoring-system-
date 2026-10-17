function [soc_next, Vcells, Snext, Ibleed, Pbleed] = balancer(soc_prev, S_prev)
% balancer  - discrete step passive balancing logic for N cells
% Inputs:
%   soc_prev : 1xN vector (SOC of each cell, 0..1)
%   S_prev   : 1xN vector (previous bleed switch states 0/1)
% Outputs:
%   soc_next : 1xN vector (updated SOC)
%   Vcells   : 1xN vector (OCV computed from SOC)
%   Snext    : 1xN vector (next switch states)
%   Ibleed   : 1xN vector (bleed currents in A)
%   Pbleed   : 1xN vector (bleed power in W)

% ---------------- Parameters (edit if needed) ----------------
N = 12;                    % number of cells
Q_Ah = 3.3;                % Ah
Q_C = Q_Ah * 3600;         % Coulombs
R_shunt = 40;              % Ohm (bleed resistor)
dt = 1;                    % simulation step (s) — match Unit Delay sample time
V_hyst = 0.02;             % hysteresis (V)
% ------------------------------------------------------------

% Ensure row vectors
soc_prev = reshape(soc_prev,1,[]);
S_prev   = reshape(S_prev,1,[]);

% Compute open-circuit voltage (OCV) per cell from SOC
Vcells = ocv_from_soc(soc_prev);

% Reference voltage
Vref = mean(Vcells);

% Hysteresis logic: turn_on when >= Vref + V_hyst, turn_off when <= Vref - V_hyst
turn_on  = (Vcells >= (Vref + V_hyst));
turn_off = (Vcells <= (Vref - V_hyst));
Snext = (S_prev & ~turn_off) | turn_on;   % latch behavior

% Bleed currents: when switch ON, I = V / R_shunt
Ibleed = zeros(1,N);
Ibleed(Snext==1) = Vcells(Snext==1) ./ R_shunt;

% Update SOC: remove charge equal to Ibleed*dt (we assume no pack charge current here)
% ΔSOC = (I * dt) / (Q_C) ; subtract because bleeding discharges the cell
delta_soc = (Ibleed * dt) ./ Q_C;
soc_next = soc_prev - delta_soc;
% clamp SOC 0..1
soc_next = min(max(soc_next, 0), 1);

% Recompute Vcells based on updated SOC for output clarity (optional)
Vcells = ocv_from_soc(soc_next);

% Bleed power
Pbleed = (Vcells.^2) ./ R_shunt .* double(Snext);

end

% ---------------- helper: OCV curve (simple piecewise approximated) ----------
function Voc = ocv_from_soc(soc)
    soc = min(max(soc,0),1);
    v0 = 3.00; v10 = 3.45; v50 = 3.75; v90 = 4.05; v100 = 4.20;
    Voc = zeros(size(soc));
    for i=1:numel(soc)
        s = soc(i);
        if s < 0.10
            Voc(i) = lerp(v0, v10, s/0.10);
        elseif s < 0.50
            Voc(i) = lerp(v10, v50, (s-0.10)/0.40);
        elseif s < 0.90
            Voc(i) = lerp(v50, v90, (s-0.50)/0.40);
        else
            Voc(i) = lerp(v90, v100, (s-0.90)/0.10);
        end
    end
    % small smoothing (not necessary inside Simulink, but OK)
    % Voc = movmean(Voc, [1 1]); % disabled for coder compatibility
end

function y = lerp(a,b,x)
    y = a + (b-a).*x;
end






