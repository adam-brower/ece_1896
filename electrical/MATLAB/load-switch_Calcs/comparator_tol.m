clear
clc

extref_nom = 3.3 / 2;
extref_tol = .01;
extref_min = extref_nom * (1 - extref_tol);
extref_max = extref_nom * (1 + extref_tol);

battCut = 3.4;
ratio = extref_nom / battCut;

%C423071
R1_nom = 1600;
% C319982
R2_nom = 1500;
R_tol = 0.005;

R_1_min = R1_nom * (1 - R_tol);
R_1_max = R1_nom * (1 + R_tol);
R_2_min = R2_nom * (1 - R_tol);
R_2_max = R2_nom * (1 + R_tol);

ratio_nom = R2_nom  / (R2_nom + R1_nom);
ratio_min = R_2_min / (R_2_min + R_1_max);
ratio_max = R_2_max / (R_2_max + R_1_min);

vthresh_nom = extref_nom / ratio_nom;
vthresh_min = extref_nom / ratio_max;
vthresh_max = extref_nom / ratio_min;

% Hysteresis
R1ref = 11000;
R2ref = 11000;
Vlow = extref_nom;
Vhigh = extref_nom + 0.1;
Rhyst = R1ref * (Vlow / (Vhigh - Vlow));
