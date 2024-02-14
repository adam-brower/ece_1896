clear
clc

sum = 0;
MCU = 130;
sum = sum + MCU;
Amplifier = 2.4;
sum = sum + Amplifier;
SD = 100;
sum = sum + SD;
Pressure = 4;
sum = sum + Pressure;
Flow = 30;
sum = sum + Flow;
Altitude = 2; 
sum = sum + Altitude;
boost_eff = 0.8;
Boost = 0.01 * (1/boost_eff);
sum = sum + Boost;
OLED = 20;
sum = sum + OLED;
LEDs = 60;
sum = sum + LEDs;
headroom = 0;
sum = sum + headroom;

disp(['Worst-case current draw: ' num2str(sum) ' mA'])
