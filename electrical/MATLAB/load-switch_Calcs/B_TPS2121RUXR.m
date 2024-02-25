clear
clc

extref_nom = 3.3;
extref_tol = .01;
extref_min = extref_nom * (1 - extref_tol);
extref_max = extref_nom * (1 + extref_tol);

battCut = 3.5;
ratio = extref_nom / battCut;

R1 = 200;
R2 = 5600;
resRatio_nom = (R2) / (R1 + R2);
resTol = 0.01;

ratioErrorPer = ((resRatio_nom - ratio) / (ratio)) * 100;

disp(['Nominal Resistor voltage divider percent error: ' num2str(ratioErrorPer) '%']);

R1_min = R1 * (1 - resTol);
R1_max = R1 * (1 + resTol);
R2_min = R2 * (1 - resTol);
R2_max = R2 * (1 + resTol);

resRatio_min = (R2_min) / (R1_max + R2_min);
resRatio_max = (R2_max) / (R1_min + R2_max);

switchPoint_nom = extref_nom / resRatio_nom;
switchPoint_min = extref_min / resRatio_max;
switchPoint_max = extref_max / resRatio_min;

disp(['Lowest possible switching point with resistor and VDD tolerance: ' num2str(switchPoint_min) 'V']);
disp(['Highest possible switching point with resistor and VDD tolerance: ' num2str(switchPoint_max) 'V']);

switchErrorPer = ((switchPoint_max - switchPoint_nom) / switchPoint_nom) * 100;

disp(['Nominal switching voltage percent error: ' num2str(switchErrorPer) '%']);

ILIM = 2.5;
ILIM_R = 10^(log10(65.2 / ILIM) / 0.861);

disp(['Required resistor to limit current output to ' num2str(ILIM) 'A: ' num2str(ILIM_R) ' kOhms']);

ILIM_R_nom = 44200;
ILIM_R_tol = 0.01;
ILIM_R_min = ILIM_R_nom * (1 - ILIM_R_tol);
ILIM_R_max = ILIM_R_nom * (1 + ILIM_R_tol);

ILIM_max = 65.2 / ((ILIM_R_min / 1000) ^ 0.861);
ILIM_min = 65.2 / ((ILIM_R_max / 1000) ^ 0.861);

disp(['Lowest Possible Output Current Limit: ' num2str(ILIM_min) ' A']);
disp(['Highest Possible Output Current Limit: ' num2str(ILIM_max) ' A']);

ILIM_ErrorPer = abs(((ILIM_min - ILIM) / ILIM) * 100);

disp(['Nominal current limit percent error: ' num2str(ILIM_ErrorPer) '%']);


