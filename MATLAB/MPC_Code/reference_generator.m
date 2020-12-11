function [Y_ref, psi_ref] = reference_generator( X )

shape = 2.4; 
d_x_1 = 25;
d_x_2 = 21.95; 
d_y_1 = 4.05; 
d_y_2 = 5.7; 
X_s_1 = 27.19; 
X_s_2 = 56.46;

z_1 = (shape / d_x_1) * (X-X_s_1) - (shape / 2);
z_2 = (shape / d_x_2) * (X-X_s_2) - (shape / 2);

Y_ref = ((d_y_1 / 2) * (1 + tanh(z_1)) - (d_y_2 / 2) * (1 + tanh(z_2)));

psi_ref = atan( d_y_1 * ( (1 / cosh(z_1))^2 ) * (1.2 / d_x_1) - ...
                d_y_2 * ( (1 / cosh(z_2))^2 ) * (1.2 / d_x_2));                 
   
end