%%% serial3_sindy.m - Main File for System Identification using SINDy
%%% on Serial3 Robot Simulation
%%% This file is designed to load the simulation data from Simulink, 
%%% generate the necessary data for system identification using SINDy, 
%%% and visualize the results of the system identification process.

%%% Author: [Juan C. García-Hernández]
%%% Date: [2026-05-05]

%% Load Simulation Data from Simulink model
%%% This section is responsible for loading the simulation data 
%%% generated from the Simulink model. 
%%% Each variable corresponds to the joint positions, velocities, 
%%% accelerations, and torques for the three joints of 
%%% the Serial3 robot.

%%% Position data 
q1s = out.q1_sim.Data;
q2s = out.q2_sim.Data;
q3s = out.q3_sim.Data;

%%% Velocity data
qp1s = out.qp1_sim.Data;
qp2s = out.qp2_sim.Data;
qp3s = out.qp3_sim.Data;

%&& Acceleration data
qpp1s = out.qpp1_sim.Data;
qpp2s = out.qpp2_sim.Data;
qpp3s = out.qpp3_sim.Data;

%&& Torque data
tau1s = out.qt_tau.Data(:,1);
tau2s = out.qt_tau.Data(:,2);
tau3s = out.qt_tau.Data(:,3);

%%% Time vector for graphing.
tt = out.q1_sim.Time;

%% SINDy Algorithm for System Identification
%%% This section define the parameters for the SINDy algorithm, 
%%% builds the library of nonlinear time series, 
%%% and performs sparse regression to identify the underlying dynamics 
%%% of the system based on the simulation data.

polyorder = 4;  % Search space up to fifth order polynomials
usesine = 0;    % No trig functions
n = 3;          % 3 joints (number of target columns refit by sparsifyDynamics)
s_t1s = size(tau1s); % Verify the size of the torque data for joint 1.

%% Build library of the Nonlinear Time Series
%%% This section constructs the library of nonlinear time series 
%%% based on the joint positions, velocities, accelerations, and torques. 
%%% The library includes polynomial terms up to the specified order, 
%%% as well as any additional functions if specified (e.g., sine functions).

%%% Theta_qs is the library for the joint positions. 
Theta_qs = [ones(length(tau1s),1) sin(tau1s) sin(tau2s) sin(tau3s)];

%%% Theta_qs2 is the library for the joint torques.
Theta_qs2 = [ones(length(tau1s),1)...
    qpp1s qpp2s qpp3s...
    qp1s qp2s qp3s...
    q1s q2s q3s];

%%% Optional richer torque library (set useRichTorqueLibrary in
%%% serial3_dataSim.m). Adds sin/cos of the joint positions, their products
%%% with the joint accelerations and sin/cos(q2+q3), which appear in the
%%% gravity and inertia terms of the manipulator dynamics.
if ~exist('useRichTorqueLibrary','var')
    useRichTorqueLibrary = false;
end
if useRichTorqueLibrary
    qs = [q1s q2s q3s];
    qpps = [qpp1s qpp2s qpp3s];
    Theta_qs2 = [Theta_qs2 ...
        sin(qs) cos(qs) ...
        sin(qs).*qpps cos(qs).*qpps ...
        sin(q2s+q3s) cos(q2s+q3s)];
end

%%% Verify the size of the library matrices to ensure they are correctly constructed based on the input data.    
ms_t = size(Theta_qs,2);
ms2_t = size(Theta_qs2,2);

%% Compute Sparse regression: Sequential least squares
%%% This section performs sparse regression using the 
%%% sequential least squares method to identify the 
%%% underlying dynamics of the system based on the library of 
%%% nonlinear time series and the target joint positions and torques.

%%% The lambda parameter is used as a sparsification knob to control the 
%%% level of sparsity in the identified model.
%%% Lambda is our sparsification knob. 
%%% Adjusting this parameter will affect the sparsity of the 
%%% identified model, with higher values leading to sparser models and 
%%% lower values allowing for more complex models.

%%% Note that this section assumes that the function sparsifyDynamics 
%%% is defined elsewhere in the codebase, which performs the actual 
%%% sparse regression based on the input library, target data, and sparsification parameter. 
%%% Please ensure that the all the SINDy code base is added to the
%%% same path as this files.

lambda = 1e-3;      % Lambda is our sparsification knob.

%%% Threshold for the torque regression. Joint 1 torque is of order 1e-3,
%%% so the richer library uses a smaller threshold to keep its terms.
if useRichTorqueLibrary
    lambdaTau = 1e-5;
else
    lambdaTau = lambda;
end

dqs = [q1s q2s q3s]; % Joint positions as the target for the SINDy algorithm.
Xis = sparsifyDynamics(Theta_qs,dqs,lambda,n) % Perform sparse regression to identify the underlying dynamics of the system based on the joint positions.

dqs2 = [tau1s tau2s tau3s]; % Joint torques as the target for the SINDy algorithm.
Xis2 = sparsifyDynamics(Theta_qs2,dqs2,lambdaTau,n) % Perform sparse regression to identify the underlying dynamics of the system based on the joint torques.

%% Compute the identified dynamics based on the library and the sparse regression results. This will give us the predicted joint positions and torques based on the identified model from SINDy, which can then be compared to the original simulation data for validation.
%%% This section computes the identified dynamics based on the 
%%% library and the sparse regression results. 
%%% This will give us the predicted joint positions and torques 
%%% based on the identified model from SINDy, 
%%% which can then be compared to the original simulation data for validation.

ds_qs = Theta_qs*Xis;  % Predicted joint positions based on the identified model from SINDy.
ds2_qs = Theta_qs2*Xis2; % Predicted joint torques based on the identified model from SINDy.

%% Graphical Comparison of SINDy Results with Simulation Data
%%% This section visualizes the results of the SINDy algorithm 
%%% by plotting the predicted joint positions and torques against 
%%% the original simulation data. 
%%% This allows us to assess the accuracy of the identified model 
%%% and understand how well it captures the dynamics of the Serial3 robot.

figure(1)
plot(tt,ds_qs(:,1),'--')
hold on
plot(tt,q1s)
title("Comparativa #1");
grid on
legend("SINDy","Simulacion",Location="southeast")
ylabel("\theta_1 (q1)");
xlabel("Datos");

figure(2)
plot(tt,ds_qs(:,2),'--')
hold on
plot(tt,q2s)
title("Comparativa #2");
grid on
legend("SINDy","Simulacion",Location="southeast")
ylabel("\theta_2 (q2)");
xlabel("Datos");

figure(3)
plot(tt,ds_qs(:,3),'--')
hold on
plot(tt,q3s)
title("Comparativa #3");
grid on
legend("SINDy","Simulacion",Location="southeast")
ylabel("\theta_3 (q3)");
xlabel("Datos");

%% Graphical Comparison of SINDy Results with Simulation Data - Torque Comparison
%%% This section visualizes the results of the SINDy algorithm 
%%% by plotting the predicted joint torques against the original 
%%% simulation data. 
%%% This allows us to assess the accuracy of the identified model in 
%%% capturing the dynamics of the Serial3 robot in terms of the torques required for the joint movements.

figure(4)
plot(tt,ds2_qs(:,1),'--')
hold on
plot(tt,tau1s)
title("Comparativa #4");
grid on
legend("SINDy","Simulacion",Location="southeast")
ylabel("\tau_1 (tau1)");
xlabel("Datos");

figure(5)
plot(tt,ds2_qs(:,2),'--')
hold on
plot(tt,tau2s)
title("Comparativa #2");
grid on
legend("SINDy","Simulacion",Location="southeast")
ylabel("\tau_2 (tau2)");
xlabel("Datos");

figure(6)
plot(tt,ds2_qs(:,3),'--')
hold on
plot(tt,tau3s)
title("Comparativa #3");
grid on
legend("SINDy","Simulacion",Location="southeast")
ylabel("\tau_3 (tau3)");
xlabel("Datos");