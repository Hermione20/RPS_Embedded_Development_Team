% clc;
% clear;

% syms k1 k2 k3 w P bT_gain Vmax Kv Pmax T;

T = balance_chassisDriving_Encoder0Torque;
w = balance_chassisDriving_Encoder0rate_rpm;
P = capacitance_messageout_power/100/2;

k1 = -0.0496;
k2 = -0.5992;
k3 = 20.5621;
 Ptest = k1.*(T.^2) + (w.*T./9.55) + k2.*(w.^2) + k3;

% matlabFunction(P,'File','all_power_cal');


% err = 2*Vmax;
% T = bT_gain + err*Kv;
% f = k1*(T^2) + (w*T/9.55) + k2*(w^2) + k3 - Pmax
% Vmax = solve(f,Vmax)
% matlabFunction(Vmax,'File','Vmax_cal');



 %clc,clear;

% KT = 0.3/(3591/187);
%KE = (60/(2*pi*24.48))/(3591/187);
%KT = 0.3/14;
%KE = (60/(2*pi*24.48))/14;

% bus_voltage = 24.56;
% give_toque_current = 20 * give_current / 16384;
% give_toque = KT .* give_toque_current;
% input_power = bus_voltage * current;

input_power = P;

machine_power = (w .* T) / 9.550;

% figure;
% plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),T);
% hold on;
% plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),motor_chassis0given_current);
% legend('Give current', 'Given current'); 
% figure;



figure;
plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),machine_power);
hold on;
plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),input_power);
legend('Machine power', 'Input power'); 




Pother = input_power - machine_power;
g = fittype('k1*w^2+k2*T.^2+c','independent',{'w','T'}, ...
    'dependent','Pother','coefficients',{'k1','k2','c'});
myfit = fit([w,T],Pother,g);
% Pre_Pother = 2.11e-07   *motor_chassis0speed_rpm.^2+ 9.805e-08 *give_current.^2 +  2.138;
%csv4
% Pre_Pother = 1.23e-07   *motor_chassis0speed_rpm.^2+ 1.453e-07 *give_current.^2 +  4.081;
%csv2
% Pre_Pother = 2.046e-07   *motor_chassis0speed_rpm.^2+ 1.488e-07 *give_current.^2 +  2.156;
%14空载
%Pre_Pother = -9.282e-08   *motor_chassis0speed_rpm.^2 -8.905e-07 *give_current.^2 +  0.6845;
%14 摩擦轮
%Pre_Pother = -1.182e-07   *motor_chassis0speed_rpm.^2+2.801e-07 *give_current.^2   -0.429;
%14_3 摩擦轮
 Pre_Pother = 0.000494  *w.^2-2.528 *T.^2  +1;


figure;
plot(linspace(1,size(Pre_Pother,1),size(Pre_Pother,1)),Pre_Pother);
hold on;
plot(linspace(1,size(Pre_Pother,1),size(Pre_Pother,1)),Pother);
predicte_power = machine_power + Pre_Pother;
legend('other power', 'predicte other power'); 


figure;
plot(linspace(1,size(input_power,1)/500,size(input_power,1)),input_power);
hold on;
plot(linspace(1,size(predicte_power,1)/500,size(predicte_power,1)),predicte_power);
legend('real power', 'predicte power'); 

