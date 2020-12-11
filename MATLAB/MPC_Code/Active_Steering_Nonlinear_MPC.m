clc; 
clear all; 
close all;
%%

%%
% Settings for simulation time
v = 10; % speed of vehicle in m/s, this is usually applied in the kinematic bycicle model
vx = v; % longitudinal speed used in full car model.

Np = 7; % Output horizon 
Nc = 2; % Output horizon 
SimTime = 0.0;
t = 1;
dt = 0.1; %simulation time in seconds

%Physical parameters of Car
m = 1700; % in Kg
Iz = 2900; % from 
a = 1.5; lf = a; %length from CoG to front axle in meters
b = 1.4; lr = b; %length from CoG to back axle in meters

%%
%TIRE MODEL
Fz = (1/2) * (m * 9.81) / 1000; %force in K-newtons
a1 = -22.1; a2 = 1011; a3 = 1078; a4 = 1.82; a5 = 0.208; 
a6 = 0.000; a7 = -0.354; a8 = 0.707; 

C = 1.30;
D =  1.0 * m * 9.81 / 2.0; %a1 * (Fz^2) + a2 * Fz; 
% BCD = a3 * sin(a4 * atan(a5 * Fz)); 
B = 1.0; % BCD / (C * D); 
%E = a6 * (Fz^2) + a7 * Fz + a8;

%%
%Declare State and Input Variables
xi = sdpvar(5,Np,'full'); % the oder is xi = [X, Y, PSI, VY, r]'
u = sdpvar(1,Nc,'full'); % Delta
%declare the starting state zo. 
xi_temp(:,1) = [0 0 0 0 0]'; % STARTING POINT

%%

Time = [SimTime];
Logs = [];

while( SimTime < 12)
    
    
    %Initial z value goes into constrains
    Constr = [xi(:,1) == xi_temp(:,t)];
    %%
    %Initial constrainst
     for i= 1:1:Np
            Constr = [Constr ];
     end
    %% ADD constraints
      
    for j = 1:1:Nc-1
        
        alphaF = u(1,j) - atan2((xi(4,j)+ lf * xi(5,j)),vx ) ;
        alphaR = -atan2((xi(4,j) - xi(5,j) * lr) , vx );
        
        FyF = D * sin(C * atan(B * alphaF));
        FyR = D * sin(C * atan(B * alphaR));
        
        Constr = [Constr, ...
        %DYNAMIC EQUATIONS OF THE CAR
        xi(1,j+1) == xi(1,j) + (vx * cos(xi(3,j)) - xi(4,j) * sin(xi(3,j))) *dt, ... %X
        xi(2,j+1) == xi(2,j) + (vx * sin(xi(3,j)) + xi(4,j) * cos(xi(3,j))) *dt,... %Y
        xi(3,j+1) == xi(3,j) + xi(5,j) * dt,... %psi => yaw angle
        xi(4,j+1) == xi(4,j) + ((1/m) * (FyF * cos(u(1,j)) + FyR) - vx * xi(5,j)) *dt,... %vy
        xi(5,j+1) == xi(5,j) + (1 / Iz) * ( lf * FyF * cos(u(1,j)) - lr * FyR) * dt,... %r => yaw rate
        
        %% Vy constraints
        %-5 <= xi(4,j+1) <= 5, ...%20m/s =72km/h
        
        %% Input constraints
        -30*pi/180 <= u(1,j+1) - u(1,j) <= 30*pi/180, ...
        
        -20*pi/180 <= u(1,j) <= 20*pi/180, ...
        ];
    end
    
    for j = Nc:1:Np-1
        
        alphaF = u(1,Nc-1) - atan2((xi(4,j)+ lf * xi(5,j)),vx ) ;
        alphaR = -atan2((xi(4,j) - xi(5,j) * lr) , vx );
        FyF = D * sin(C * atan(B * alphaF));
        FyR = D * sin(C * atan(B * alphaR));
        
        Constr = [Constr,...
        
        %DYNAMIC EQUATIONS OF THE CAR
        xi(1,j+1) == xi(1,j) + (vx * cos(xi(3,j)) - xi(4,j) * sin(xi(3,j))) * dt,...  %X
        xi(2,j+1) == xi(2,j) + (vx * sin(xi(3,j)) + xi(4,j) * cos(xi(3,j))) * dt,... %Y
        xi(3,j+1) == xi(3,j) + xi(5,j) * dt,... %psi => yaw angle
        xi(4,j+1) == xi(4,j) + ((1 / m) * (FyF * cos(u(1,Nc-1)) + FyR) - vx * xi(5,j)) *dt,... %vy
        xi(5,j+1) == xi(5,j) + (1 / Iz) * ( lf * FyF * cos(u(1,Nc-1)) - lr * FyR) * dt  %r => yaw rate
        
        %-5 <= xi(4,j+1) <= 5, ...%20m/s =72km/h
        ];
    end
    
    %% CREATE REFRENCE
    % z_ref = getref(xi_temp(1,t),xi_temp(2,t),dt,N,DriftRadius,vx);
        
    %%
    % COST FUNCTION
    [ref_Y, ref_psi] = reference_generator(xi_temp(1,t));
    
    ref_Y_Log(t+1) = ref_Y;
    ref_psi_Log(t+1) = ref_psi; 
    
    Cost = 0;
    
    for k = 1:Np
        
        [ref_Y, ref_psi] = reference_generator( xi_temp(1,t) + v * (k - 1) * dt );
        
        Cost = Cost + 5 * (ref_Y - xi(2,k))^2 + ...
                      100 * (ref_psi - xi(3,k))^2; 
        
    end
    
    for k = 1:Nc-1
        
        Cost = Cost + 0.1*(u(1,k+1) - u(1,k))^2;
        
    end
    %%
    %Solve Optimization Problem
    options = sdpsettings('solver', 'fmincon', 'verbose', 0, 'debug', 0);
    %options = sdpsettings('solver', 'cplex', 'verbose', 0, 'debug', 0);
    %options = sdpsettings('solver', 'gurobi', 'verbose', 0, 'debug', 0);
    solution =  solvesdp(Constr,Cost,options);
    
    %display a timer for ease of reading
    disp(['t= ',num2str(t * dt),' of ',num2str(SimTime),...
          ' seconds in increments of ',num2str(dt),' seconds']); %visual timer
    %%
    %use the optmized U of the solver and save them.
    u_star(:,t) = double(u(:,1)); %Use only the first value of the optimized solution
    zopen{1,t} = double(xi);
        
    %Update the Alfas and Forces.
    alphaF(t) = double( u_star(1,t) - atan2( (xi_temp(4,t) + lf * xi_temp(5,t)), vx));
    alphaR(t) = double( atan2((lr * xi_temp(5,t)- xi_temp(4,t)) , vx ) );
    FyF(t) = D * sin(C * atan(B * alphaF(t))); 
    FyR(t) = D * sin(C * atan(B * alphaR(t))); 
    
    % the first value of the optimized U + the z to predict the next z.
    xi_temp(1,t+1) = xi_temp(1,t) + (vx * cos(xi_temp(3,t)) - xi_temp(4,t) * sin(xi_temp(3,t))) *dt; %X
    xi_temp(2,t+1) = xi_temp(2,t) + (vx*sin(xi_temp(3,t)) + xi_temp(4,t) * cos(xi_temp(3,t))) * dt; %Y
    xi_temp(3,t+1) = xi_temp(3,t) + xi_temp(5,t)*dt; % psi => yaw angle
    xi_temp(4,t+1) = xi_temp(4,t) + ((1 / m) * (FyF(t) * cos(u_star(1,t)) + FyR(t)) - vx * xi_temp(5,t)) *dt; %vy
    xi_temp(5,t+1) = xi_temp(5,t) + (1 / Iz) * ( lf * FyF(t) * cos(u_star(1,t)) - lr * FyR(t)) * dt; %r => yaw rate
    
   
    %%
    %PLOT OF X VS Y, SIMULATION MPC VS REFERENCE
    plot(xi_temp(1,:),xi_temp(2,:),'b--o','LineWidth',2); hold on; %Plot open loop
    % circle(0,0,DriftRadius) % Plot Circl trajectory
    % plot(z_ref(1,:),z_ref(2,:),'r--o'); hold off;
    title('X vs Y MPC SIMULATION');
    %%axis([-10,60,-52,10]);
    
    %%
    
    Logs = [Logs; double(xi_temp(3,t+1) - xi_temp(3,t)),...
                  double(xi_temp(2,t+1) - xi_temp(2,t)),...
                  double(alphaF(1,t)),...
                  double(alphaR(1,t)),...
                  double(FyF(1,t)),...
                  double(FyR(1,t)),...
                  double(solution.solvertime)];  
    
    u_star(1,t);
    t = t + 1;
    SimTime = SimTime + dt;
    Time = [Time; SimTime];
end

Plotter;
