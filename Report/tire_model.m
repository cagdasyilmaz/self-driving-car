clc;
clear all;
close all;

%%Physical parameters of Car
m=1700; % in Kg
Iz=2900; % 
a=1.5; %length from CoG to front axle in meters
b=1.4; %length from CoG to back axle in meters


%% Pajecka Tire Model (Magic Formula) 

%Slip angle takes values between -20 and 20 degrees.
%Slip ratio takes values between -100% and 100%.
a1_x=-21.3; a2_x=1144; a3_x=49.6; a4_x=226; a5_x=0.069; a6_x=-0.006; 
a7_x=0.056; a8_x=0.486;

C_x=1.65;


a1_y=-22.1; a2_y=1011; a3_y=1078; a4_y=1.82; a5_y=0.208; a6_y=0.0; 
a7_y=-0.354; a8_y=0.707;

C_y=1.3;
%%

Fz_f = (b*m*9.81)/(1000*2*(a+b)); %force in K-newtons
Fz_r = (a*m*9.81)/(1000*2*(a+b)); %force in K-newtons

Fz = [Fz_f; Fz_r];

j = 0;
for k = 1:1:2
 
 i=0;
 j=j+1;
 
 for alpha = -20:0.01:20
 i=i+1;
 
 D_x = (a1_x * (Fz(k)^2)) + ( a2_x * Fz(k) );
 BCD_x = ((a3_x * (Fz(k)^2)) + (a4_x * (Fz(k)^2))) / (exp(a5_x * Fz(k)));
 B_x = BCD_x / (C_x * D_x);
 E_x = (a6_x * (Fz(k)^2)) + a7_x * Fz(k) + a8_x;
 
 D_y = (a1_y * (Fz(k)^2)) + (a2_y * Fz(k));
 BCD_y = a3_y * sind(a4_y * atand(a5_y * Fz(k)));
 B_y = BCD_y / (C_y * D_y);
 E_y = (a6_y * (Fz(k)^2)) + a7_y * Fz(k) + a8_y;
 
 
 Fl(i,j) = D_x * sind(C_x * atand(B_x * (alpha)));
 Fc(i,j) = D_y * sind(C_y * atand(B_y * (alpha)));
 end
 
end
%%

ratio = -100:0.05:100;
figure
plot(ratio , Fl(:,1), ratio, Fl(:,2), 'linewidth', 2)
grid on
title('$Longitudinal\,\,Forces\,\,for\,\,Front\,\,and\,\,Rear\,\,Tyres$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Tire\,\,slip\,\,ratio[\%]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$F_{l_{f}},F_{l_{r}}[N]$','fontsize', 18, 'interpreter', 'latex')
l = legend('$F_{l_{f}}$','$F_{l_{r}}$','Location','SouthEast');
set(l, 'interpreter', 'latex', 'fontsize', 18)


alpha =-20:0.01:20;
figure
plot(alpha, Fc(:,1), alpha, Fc(:,2), 'linewidth', 2)
grid on
title('$Lateral\,\,Forces\,\,for\,\,Front\,\,and\,\,Rear\,\,Tyres$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Tire\,\,slip\,\,angle[deg]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$F_{c_{f}},F_{c_{r}}[N]$','fontsize', 18, 'interpreter', 'latex')
l = legend('$F_{c_{f}}$','$F_{c_{r}}$','Location','SouthEast');
set(l, 'interpreter', 'latex', 'fontsize', 18)
