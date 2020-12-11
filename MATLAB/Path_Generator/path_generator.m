clear;
clc;
close all;

shape = 2.4; 
d_x_1 = 25;
d_x_2 = 21.95; 
d_y_1 = 4.05; 
d_y_2 = 5.7; 
X_s_1 = 27.19; 
X_s_2 = 56.46;

X = 0:0.05:120;

z_1 = (shape/d_x_1).*(X-X_s_1) - (shape/2);
z_2 = (shape/d_x_2).*(X-X_s_2) - (shape/2);

Y_ref = ((d_y_1/2)*(1+tanh(z_1)) - (d_y_2/2)*(1+tanh(z_2)));
psi_ref = [];

for i = 1:length(z_1)
   psi_ref_instant = (180/pi)*atan( d_y_1 * ( (1/cosh(z_1(i)))^2 ) * (1.2/d_x_1) -d_y_2 * ( (1/cosh(z_2(i)))^2 ) * (1.2/d_x_2));                 
   psi_ref = [psi_ref, psi_ref_instant];
end

plot(X, Y_ref, 'LineWidth', 2);
grid on
xlabel('$x\;\;position[m]$','fontsize', 18, 'interpreter', 'latex' )
ylabel('$y\;\;position[m]$','fontsize', 18, 'interpreter', 'latex')
l = legend('$Reference\;\;\hat{Y}\;\;Position$');
set(l, 'interpreter', 'latex', 'fontsize', 18) 

figure
plot(X, psi_ref, 'LineWidth', 2);
grid on
xlabel('$x\;\;position[m]$','fontsize', 18, 'interpreter', 'latex' )
ylabel('$\psi\;\;angle[deg]$','fontsize', 18, 'interpreter', 'latex')
l = legend('$Reference\;\;\hat{\psi}\;\;Angle\;\;[deg]$');
set(l, 'interpreter', 'latex', 'fontsize', 18) 