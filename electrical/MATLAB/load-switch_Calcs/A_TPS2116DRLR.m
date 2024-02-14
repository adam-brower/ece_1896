clear 
clc

R1 = 6200;
R2 = 2700;

tolper = 0.5;
tol = tolper / 100;

R1_min = R1 * (1 - tol);
R1_max = R1 * (1 + tol);
R2_min = R2 * (1 - tol);
R2_max = R2 * (1 + tol);

scale_nom = (R2)   / (R2 + R1);
scale_1 = (R2_min) / (R2_min + R1_min);
scale_2 = (R2_min) / (R2_min + R1_max);
scale_3 = (R2_max) / (R2_max + R1_min);
scale_4 = (R2_max) / (R2_max + R1_max);

Vref = 1;

vbatt_trig_nom = Vref / scale_nom;
vbatt_trig_1 = Vref / scale_1;
vbatt_trig_2 = Vref / scale_2;
vbatt_trig_3 = Vref / scale_3;
vbatt_trig_4 = Vref / scale_4;

disp(['Nominal cell voltage to trip switch: ' num2str(vbatt_trig_nom) 'V']);
disp(['Case 1: cell voltage to trip switch: ' num2str(vbatt_trig_1) 'V']);
disp(['Case 2: cell voltage to trip switch: ' num2str(vbatt_trig_2) 'V']);
disp(['Case 3: cell voltage to trip switch: ' num2str(vbatt_trig_3) 'V']);
disp(['Case 4: cell voltage to trip switch: ' num2str(vbatt_trig_4) 'V']);

Vref_min = 0.92;
Vref_max = 1.08;

vbatt_trig_nom = Vref_min / scale_nom;
vbatt_trig_1 = Vref_min / scale_1;
vbatt_trig_2 = Vref_min / scale_2;
vbatt_trig_3 = Vref_min / scale_3;
vbatt_trig_4 = Vref_min / scale_4;

disp(" ");
disp('VREF Min:');
disp(['Nominal cell voltage to trip switch: ' num2str(vbatt_trig_nom) 'V']);
disp(['Case 1: cell voltage to trip switch: ' num2str(vbatt_trig_1) 'V']);
disp(['Case 2: cell voltage to trip switch: ' num2str(vbatt_trig_2) 'V']);
disp(['Case 3: cell voltage to trip switch: ' num2str(vbatt_trig_3) 'V']);
disp(['Case 4: cell voltage to trip switch: ' num2str(vbatt_trig_4) 'V']);

vbatt_trig_nom = Vref_max / scale_nom;
vbatt_trig_1 = Vref_max / scale_1;
vbatt_trig_2 = Vref_max / scale_2;
vbatt_trig_3 = Vref_max / scale_3;
vbatt_trig_4 = Vref_max / scale_4;

disp(" ");
disp('VREF Max:');
disp(['Nominal cell voltage to trip switch: ' num2str(vbatt_trig_nom) 'V']);
disp(['Case 1: cell voltage to trip switch: ' num2str(vbatt_trig_1) 'V']);
disp(['Case 2: cell voltage to trip switch: ' num2str(vbatt_trig_2) 'V']);
disp(['Case 3: cell voltage to trip switch: ' num2str(vbatt_trig_3) 'V']);
disp(['Case 4: cell voltage to trip switch: ' num2str(vbatt_trig_4) 'V']);
