clear
clc

vout = 5;
P_flow = 0.150;      % Flow sensor sucks 150mW
iout = P_flow / vout;

eff = 0.8;          % Boot efficiency

vin_min = 3.3;      % minimum input voltage
vin_max = 4.2;      % Maximum input voltage
vin_avg = 3.7;      % Avg. Cell voltage

ccm_min = 2 * (vout * iout) / (vin_min * eff);
ccm_max = 2 * (vout * iout) / (vin_max * eff);
ccm_avg = 2 * (vout * iout) / (vin_avg * eff);

disp(['Max. ripple I for min. battery voltage: ' num2str(1000*ccm_min) 'mA']);
disp(['Max. ripple I for max. battery voltage: ' num2str(1000*ccm_max) 'mA']);
disp(['Max. ripple I for avg. battery voltage: ' num2str(1000*ccm_avg) 'mA']);

minRipp = min([ccm_min, ccm_avg, ccm_max]);

% disp(['Min. ripple I to design for: ' num2str(1000*minRipp) 'mA']);


