 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lab
% Position and Kinodynamic control
% Author: Giuseppe Anastasio (giuseppe1.anastasio@mail.polimi.it)
% 25/01/2025     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear;
close all;
clc;
format long

addpath common 

% %% SIMULATION SETUP
% FLAG_act = 0;
% FLAG_sens = 0;
% scenario = 2;  % circularTrajectory: 2      setpointTrajectory: 1
% 
% % Auxiliary variables
% degToRad = pi/180;
% radTodeg = 180/pi;
% par.g = 9.81; %m/s^2
% par.e_1 = [1 0 0]'; 
% par.e_2 = [0 1 0]'; 
% par.e_3 = [0 0 1]'; 
% 
% 
% % Initial conditions
% if scenario == 1
%     v_0 = [0 0 0]'; % m/s linear velocity
%     p_0 = [0 0 0]'; % m position
%     stop_time = '100';
% elseif scenario == 2
%     v_0 = [pi/10*cos(0); -pi/10*sin(0); 0];
%     p_0 = [3*sin(0); 3*cos(0); -2];
%     stop_time = '3000';
% end
% 
% %%% UAV inertial and dynamic parameters
% 
% UAV.m = 0.31; %[kg] UAV mass
% UAV.mass = UAV.m;
% UAV.J = [0.00021     0        0;
%                0      0.00024        0;
%                0        0      0.00032]; % [kg m^2] inertia matrix
% UAV.inertia = UAV.J;
% UAV.r_bg = [0 0 0.02]; %[m] center of mass location in the body frame
% UAV.S = UAV.m*crossmat(UAV.r_bg) ; % [kg m] static moment
% UAV.static = UAV.S;
% UAV.M = [UAV.m*eye(3) UAV.S' ; % generalized mass matrix
%          UAV.S UAV.J] ;
% UAV.Minv = inv(UAV.M) ; % inverse of generalized mass matrix
% UAV.Jinv = inv(UAV.J); % inverse of inertial matrix
% 
% m = UAV.m;
% 
% % Linear Aerodynamics
% D_fv = diag([0.07 0.07 0.1]); % linear velocity damping
% D_fv = diag([0.05;0.08;0.08]); % linear velocity damping
% 
% % Actuators effectiveness
% lambda = 1;
% lambda = 1;
% 
% %%% CONTROL PARAMETERS
% 
% ctrl.vMax = 10;
% 
% % NEW PID CONTROLLER
% % Outer loop
% ctrl.Kpos = 0.5*diag([1,1,1]);  % 0.5
% ctrl.Kipos = 0*diag([1,1,1]);
% ctrl.Kdpos = 0*diag([1,1,1]);
% % Inner loop
% ctrl.Kv = 0.1*diag([1,1,1]);  % 1
% ctrl.Kiv = 0.1*diag([1,1,1]);  % 0.2
% ctrl.Kdv = 0*diag([1,1,1]);
% 
% % OLD CASCADE CONTROLLER
% % Outer loop
% ctrl.Kpos = 1*diag([1,1,1]);  % 0.5
% ctrl.Kipos = 0*diag([1,1,1]);
% ctrl.Kdpos = 0*diag([1,1,1]);
% % Inner loop
% ctrl.Kv = 1*diag([1,1,1]);  % 1
% ctrl.Kiv = 0.1*diag([1,1,1]);  % 0.2
% ctrl.Kdv = 0*diag([1,1,1]);
% 
% ctrl.v_M = 10;
% ctrl.Kp = ctrl.Kpos;
% 
% %%% Target trajectory for setpoint tracking
% 
% targetParams.omega_p_dot = zeros(3,1);
% targetParams.v_b_dot = zeros(3,1);
% targetParams.v_b = zeros(3,1);
% targetParams.p = [1 2 -3]';
% targetParams.heading = pi/4;
% 
% %%% Adaptive controller parameters
% 
% adaptiveActivation = 1;
% gravityInAdaptiveController = 1;
% Gamma_a = 1*[1 1 1 1]';   % 1 without gravity
% Gamma_a_scalar = 1;
% L = 10;
% %L = [5;5;5];  % 5 5 2 without gravity
% lambda_0 = 1;
% theta_0 = [0.07;0.07;0.1];
% B_p = 1/UAV.m;
% 
% 
% PO = 1;
% projection_operator = PO;
% par.Gamma = Gamma_a;
% par.eps_proj = 0.01;
% epsilon_PO = par.eps_proj*[1;1;1;1];
% par.theta_M = [0.8; 0.035; 0.035; 0.05];
% theta_M = par.theta_M;
% par.center = [lambda_0; theta_0];
% 
% 
% %%% Stupid stuff
% Environment.g = par.g;
% gravity = par.g;
% Environment.e_1 = par.e_1; Environment.e_2 = par.e_2; Environment.e_3 = par.e_3;
% e3 = Environment.e_3;
% g = par.g;
% m = UAV.m;
% D_fv = UAV.D_fv;
% v_0 = vb_0;




%% SIMULATION SETUP
FLAG_act = 0;
FLAG_sens = 0;
scenario = 2;  % circularTrajectory: 2      setpointTrajectory: 1

% auxiliary variables
degToRad = pi/180;
radTodeg = 180/pi;
par.g = 9.81; %m/s^2
par.e_1 = [1 0 0]'; par.e_2 = [0 1 0]'; par.e_3 = [0 0 1]';
Environment.g = par.g;
gravity = par.g;
Environment.e_1 = par.e_1; Environment.e_2 = par.e_2; Environment.e_3 = par.e_3;
e_3 = Environment.e_3;

% Initial conditions
if scenario == 1
    q_0 = eul2quat([0, 0, 0 ]*degToRad,'ZYX')'; %  attitude - quaternion
    omegab_0 = [0 0 0]'; %  rad/s  angular velocity (body components)
    vb_0 = [0 0 0]'; % m/s linear velocity
    p_0 = [0 0 0]'; % m position
    Omega_0 = [1500 1500 1500 1500]';
    stop_time = '10';
elseif scenario == 2
    q_0 = eul2quat([0, 0, 0]*degToRad,'ZYX')'; %  attitude - quaternion
    omegab_0 = [0; 0; 0]; % 2*pi/60 0 0 
    vb_0 = [0; 0; 0]; % pi/10*[cos(0); -sin(0); 0]
    p_0 = [0; 0; 0];
    Omega_0 = [1500 1500 1500 1500]';
    stop_time = '500';
end



attitude_init = q_0; % Initial attitude - quaternion
omega_b_init = omegab_0; %  rad/s Initial angular velocity (body components)
pos_init = p_0; %m
vel_b_init = vb_0; %m/s
vel_state_init = [vel_b_init; omega_b_init];


%%% UAV inertial and dynamic parameters

% Nominal values for the parameters
UAV.m = 0.31; %[kg] UAV mass
UAV.mass = UAV.m;
UAV.J = [0.00021     0        0;
               0      0.00024        0;
               0        0      0.00032]; % [kg m^2] inertia matrix
UAV.inertia = UAV.J;
UAV.r_bg = [0 0 0]; %[m] center of mass location in the body frame
UAV.S = UAV.m*crossmat(UAV.r_bg) ; % [kg m] static moment
UAV.static = UAV.S;
UAV.M = [UAV.m*eye(3) UAV.S' ; % generalized mass matrix
         UAV.S UAV.J] ;
UAV.Minv = inv(UAV.M) ; % inverse of generalized mass matrix
UAV.Jinv = inv(UAV.J); % inverse of inertial matrix

% Linear Aerodynamics
UAV.D_tauomega = diag([0.00033 0.00022 0.00017]); %  angular velocity damping 
UAV.D_fv =  diag([0.07 0.07 0.1]) ; % linear velocity damping
UAV.D_fv =  diag([0.1 0.04 0.11]) ; % linear velocity damping
UAV.D = [UAV.D_fv zeros(3,3); zeros(3,3) UAV.D_tauomega] ;
%UAV.D = diag([0.05 0.05 0.1  0.000333 0.0002183 0.000172]);

% Disturbance
d_tau = zeros(3,1);

% Sensor delay
UAV.sens_delay = 1;

%%% Propellers 

UAV.b = 0.08; %[m] arm length (ell in the slides)

UAV.T_max = 3; %[N] max thrust of each propeller
UAV.T_min = 0; %[N] min thrust

UAV.k_m = 70; % [s-1] Inverse of the time constant of the propeller motors
k_f = 0.3379e-6;                % [N/rad^2/s^2] Thrust characteristic coeff
sigma = 5.343e-3;      % [m] Torque-to-thrust ratio

lambda = 0.89;
UAV.lambda_r = [lambda; lambda; lambda; lambda];

% UAV.n = [0; 0; 1] ;
% p1 = UAV.b * [ sqrt(2)/2;  sqrt(2)/2;  0] ;
% p2 = UAV.b * [-sqrt(2)/2; -sqrt(2)/2;  0] ;
% p3 = UAV.b * [ sqrt(2)/2; -sqrt(2)/2;  0] ;
% p4 = UAV.b * [-sqrt(2)/2;  sqrt(2)/2;  0] ;
% UAV.F = -[UAV.n UAV.n UAV.n UAV.n;...
%     (crossmat(p1)-sigma*eye(3))*UAV.n (crossmat(p2)-sigma*eye(3))*UAV.n...
%     (crossmat(p3)+sigma*eye(3))*UAV.n (crossmat(p4)+sigma*eye(3))*UAV.n] ;

% old F
epsilon = [-1 -1 1 1]';
UAV.p1 = UAV.b*[cos(pi/4); sin(pi/4); 0];
UAV.p2 = UAV.b*[-cos(pi/4); -sin(pi/4); 0];
UAV.p3 = UAV.b*[cos(pi/4); -sin(pi/4); 0];
UAV.p4 = UAV.b*[-cos(pi/4); sin(pi/4); 0];
UAV.F = -[par.e_3 par.e_3 par.e_3 par.e_3;
    (crossmat(UAV.p1)+epsilon(1)*sigma*eye(3))*par.e_3 (crossmat(UAV.p2)+epsilon(2)*sigma*eye(3))*par.e_3 (crossmat(UAV.p3)+epsilon(3)*sigma*eye(3))*par.e_3 (crossmat(UAV.p4)+epsilon(4)*sigma*eye(3))*par.e_3];

% new F
gamma1 = pi/4;
gamma2 = gamma1 + pi;
gamma3 = gamma2 + pi/2;
gamma4 = gamma3 - pi;
UAV.F = -[
    0 0 0 0;
    0 0 0 0;
    1 1 1 1;
    -UAV.b*sin(gamma1) -UAV.b*sin(gamma2) -UAV.b*sin(gamma3) -UAV.b*sin(gamma4);
    UAV.b*cos(gamma1) UAV.b*cos(gamma2) UAV.b*cos(gamma3) UAV.b*cos(gamma4);
    -sigma -sigma sigma sigma
    ];


%%% Allocation

% old F2
% F2 = [1 1 1 1;
%         -(crossmat(UAV.p1)+epsilon(1)*sigma*eye(3))*par.e_3 -(crossmat(UAV.p2)+epsilon(2)*sigma*eye(3))*par.e_3 -(crossmat(UAV.p3)+epsilon(3)*sigma*eye(3))*par.e_3 -(crossmat(UAV.p4)+epsilon(4)*sigma*eye(3))*par.e_3];
% 
% F2 = [1 1 1 1;
%       +UAV.b*(-sqrt(2)/2) +UAV.b*(+sqrt(2)/2) +UAV.b*(sqrt(2)/2) +UAV.b*(-sqrt(2)/2);
%       -UAV.b*(-sqrt(2)/2) -UAV.b*(+sqrt(2)/2) -UAV.b*(-sqrt(2)/2) -UAV.b*(sqrt(2)/2);
%       +sigma +sigma -sigma -sigma];

% new F2
F2 = [1 1 1 1;
    UAV.b*sin(gamma1) UAV.b*sin(gamma2) UAV.b*sin(gamma3) UAV.b*sin(gamma4);
    -UAV.b*cos(gamma1) -UAV.b*cos(gamma2) -UAV.b*cos(gamma3) -UAV.b*cos(gamma4);
    sigma sigma -sigma -sigma];

F2plus = F2'*pinv(F2*F2');

F1 = [0 0 0 0;
      0 0 0 0;
     -1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];

F1plus = pinv(F1);

%%% CONTROL PARAMETERS

ctrl.vMax = 10;

% %%% OK WITHOUT ALLOCATION AND ROTOR DYNAMICS
% % Outer loop
% ctrl.Kq = diag([0.02,0.02,0.0025]); %quaternion feedback
% ctrl.KR = diag([0.5,0.5,0.5]); %dcm feedback
% ctrl.Kpos = diag([3,3,3]); %position feedback
% % Inner loop
% ctrl.Komega = diag([0.02,0.02,0.0025]);
% ctrl.Kiomega = diag([3, 3, 0.009]);
% ctrl.Kv = diag([1,1,1]);
% ctrl.Kiv = diag([5,5,5]);
% ctrl.sample_time = 1/250; %[s]

% OLD CONTROLLER
% Outer loop
ctrl.Kq = diag([13,13,4]); %quaternion feedback
ctrl.KR = diag([15,15,15]); %dcm feedback    13,13,4
ctrl.Kpos = diag([2.9,2.9,2.5]); %position feedback   1.6*2
% Inner loop
ctrl.Komega = diag([0.002,0.002,0.001]);  % 0.0058,0.0034,0.0014
ctrl.Kiomega = diag([0.001, 0.001, 0.00007]);  % 2* 0.0013, 0.0008, 0.00009
ctrl.Kv = diag([0.48,0.48,0.59]);  % 0.9,0.9,0.9
ctrl.Kiv = diag([0.014,0.014,0.58]);  % 2* 0.4,0.4,0.4
ctrl.sample_time = 1/250; %[s]

% % RETUNED CONTROLLER
% % Outer loop
% ctrl.Kq = diag([13,13,4]); %quaternion feedback
% ctrl.KR = diag([15,15,15]); %dcm feedback    13,13,4
% ctrl.Kpos = diag([4,4,4]); %position feedback   1.6*2
% ctrl.Kipos = 0*diag([0.1,0.1,0.1]); %position feedback   1.6*2
% % Inner loop
% ctrl.Komega = diag([0.002,0.002,0.001]);  % 0.0058,0.0034,0.0014
% ctrl.Kiomega = diag([0.001, 0.001, 0.00007]);  % 2* 0.0013, 0.0008, 0.00009
% ctrl.Kv = diag([2.5,2.5,2.5]);  % 0.9,0.9,0.9
% ctrl.Kiv = 1*diag([0.1,0.1,0.58]);  % 2* 0.4,0.4,0.4
% ctrl.sample_time = 1/250; %[s]

ctrl.v_M = 10;
ctrl.Kp = ctrl.Kpos;


%%% Target trajectory for setpoint tracking

targetParams.omega_p_dot = zeros(3,1);
targetParams.v_b_dot = zeros(3,1);
targetParams.omega_p = zeros(3,1); 
targetParams.v_b = zeros(3,1);
targetParams.p = [1 1 -2]';
targetParams.heading = pi/4;


%%% Adaptive controller parameters

adaptiveActivation = 1;
targetParams.oscillation = 0;
gravityInAdaptiveController = 1;
Gamma_a = 1*[1 1 1 1]';   % 1 without gravity
Gamma_a_scalar = 1;
L = 35;  % 35
%L = [5;5;5];  % 5 5 2 without gravity
lambda_0 = 1;
theta_0 = [0.07;0.07;0.1];
B_p = 1/UAV.m;

PO = 1;
par.Gamma = Gamma_a;
par.eps_proj = 1;
par.theta_M = [0.8; 0.035; 0.035; 0.05];
par.center = [lambda_0; theta_0];

% stupid
e3 = par.e_3;
g = par.g;
%% SIMULATE MODEL

simout = sim('Simplified_UAV_Task3.slx','StopTime',stop_time);

%% PLOT RESULTS

figure
plot(simout.e_p,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[m]')
title('Position error')

figure
plot(simout.e_v,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[m/s]')
title('Velocity error')

figure
plot(simout.p,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[m]')
title('Position')
legend('N', 'E', 'D')

figure
plot(simout.vb,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[m/s]')
title('Velocity')
legend('v_N', 'v_E', 'v_D')

figure
plot(simout.u_b,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[N]')
title('Baseline controller input command')
legend('u_1', 'u_2', 'u_3')

if adaptiveActivation

    figure
    plot(simout.lambda_adaptive,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[-]')
    title('Lambda')
    legend('\lambda_1')
    
    figure
    hold on
    plot(simout.theta_adaptive,'LineWidth',3);
    yline(D_fv(1,1), "--",'LineWidth',1.5, Color="#0072BD")
    yline(D_fv(2,2), "--",'LineWidth',1.5, Color="#D95319")
    grid minor
    xlabel('[s]')
    ylabel('[-]')
    title('Theta')
    legend('\theta_1', '\theta_2', '\theta_3', 'D_{fv1}', 'D_{fv2}')

    figure
    plot(simout.e_p_hat,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[m/s]')
    title('Prediction error norm')

    figure
    plot(simout.u_a,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[N]')
    title('Adaptive controller input command')
    legend('u_1', 'u_2', 'u_3')

end

if scenario == 2

    figure
    plot(simout.posRMS,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[m]')
    title('Position RMS')

    fprintf("posRMS mean value: %f \n", mean(simout.posRMS.Data(round(length(simout.posRMS.Data)/2):end)))
    fprintf("velRMS mean value: %f \n", mean(simout.velRMS.Data(round(length(simout.velRMS.Data)/2):end)))

end


function X = crossmat(x)
X=[0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end