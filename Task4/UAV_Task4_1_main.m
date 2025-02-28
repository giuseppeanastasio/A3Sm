%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: Giuseppe Anastasio (giuseppe1.anastasio@mail.polimi.it)
%     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;

addpath common 

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
q_0 = eul2quat([0, 0, 0]*degToRad,'ZYX')'; %  attitude - quaternion
omegab_0 = [0; 0; 0];
vb_0 = [0; 0; 0];
p_0 = [0; 0; 0];
Omega_0 = [1500 1500 1500 1500]';
stop_time = '100';

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
par.fw0 = 0;
par.fw1 = 0;

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
epsilon = [-1 -1 1 1]';
UAV.p1 = UAV.b*[cos(pi/4); sin(pi/4); 0];
UAV.p2 = UAV.b*[-cos(pi/4); -sin(pi/4); 0];
UAV.p3 = UAV.b*[cos(pi/4); -sin(pi/4); 0];
UAV.p4 = UAV.b*[-cos(pi/4); sin(pi/4); 0];
UAV.F = -[par.e_3 par.e_3 par.e_3 par.e_3;
    (crossmat(UAV.p1)+epsilon(1)*sigma*eye(3))*par.e_3 (crossmat(UAV.p2)+epsilon(2)*sigma*eye(3))*par.e_3 (crossmat(UAV.p3)+epsilon(3)*sigma*eye(3))*par.e_3 (crossmat(UAV.p4)+epsilon(4)*sigma*eye(3))*par.e_3];

%%% Allocation

F2 = [1 1 1 1;
      +UAV.b*(sqrt(2)/2) +UAV.b*(-sqrt(2)/2) +UAV.b*(sqrt(2)/2) +UAV.b*(-sqrt(2)/2);
      -UAV.b*(sqrt(2)/2) -UAV.b*(-sqrt(2)/2) -UAV.b*(-sqrt(2)/2) -UAV.b*(sqrt(2)/2);
      -sigma -sigma +sigma +sigma];

F2 = [1 1 1 1;
        -(crossmat(UAV.p1)+epsilon(1)*sigma*eye(3))*par.e_3 -(crossmat(UAV.p2)+epsilon(2)*sigma*eye(3))*par.e_3 -(crossmat(UAV.p3)+epsilon(3)*sigma*eye(3))*par.e_3 -(crossmat(UAV.p4)+epsilon(4)*sigma*eye(3))*par.e_3];


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

% Outer loop
ctrl.Kq = diag([13,13,4]); %quaternion feedback
ctrl.KR = diag([13,13,4]); %dcm feedback
ctrl.Kpos = 1*diag([1.6,1.6,1.6]); %position feedback
% Inner loop
ctrl.Komega = diag([0.0058,0.0034,0.0014]);
ctrl.Kiomega = diag([0.0013, 0.0008, 0.00009]);
ctrl.Kv = 1*diag([0.9,0.9,0.9]);
ctrl.Kiv = diag([0.4,0.4,0.4]);
ctrl.sample_time = 1/250; %[s]

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
L = 100;
lambda_0 = 1;
theta_0 = [0.07;0.07;0.1];
B_p = 1/UAV.m;

PO = 1;
par.Gamma = Gamma_a;
par.eps_proj = 10;
par.theta_M = [0.8; 0.035; 0.035; 0.05];
par.center = [lambda_0; theta_0];


% stupid
e3 = par.e_3;
g = par.g;

%% SIMULATE MODEL

close all
simout = sim('UAV_Task4_12.slx','StopTime',stop_time);

%% PLOT RESULTS
close all

figure
plot(simout.quat_e,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[-]')
title('Quaternion error')
legend('eta','eps_x', 'eps_y', 'eps_z')

figure
plot(simout.e_omega,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[rad/s]')
title('Angular velocity error')

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
plot(simout.euler * radTodeg,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[deg]')
title('Attitude')
legend('\phi', '\theta', '\psi')

figure
plot(simout.omega_b,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[rad/s]')
title('Angular velocity')
legend('p', 'q', 'r')

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
plot(simout.tau_c,'LineWidth',3)
grid minor
xlabel('[s]')
ylabel('[Nm]')
title('Control torque')

if adaptiveActivation

    figure
    plot(simout.lambda_adaptive,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[-]')
    title('Lambda')
    legend('\lambda_1')
    
    figure
    plot(simout.theta_adaptive,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[-]')
    title('Theta')
    legend('\theta_1', '\theta_2', '\theta_3')

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

    fprintf("posRMS mean value after transient: %f \n", mean(simout.posRMS.Data(round(length(simout.posRMS.Data)/2):end)))
    %fprintf("velRMS mean value after transient: %f \n", mean(simout.velRMS.Data(round(length(simout.velRMS.Data)/2):end)))

end

%% Position analysis

figure
hold on
grid on
axis equal
plot(squeeze(simout.p.Data(2,1,:)),squeeze(simout.p.Data(1,1,:)));
plot(5*sqrt(2)/2, sqrt(2)/2, 'o');
plot(-5*sqrt(2)/2, -sqrt(2)/2, 'o');
plot(-sqrt(2)/2, -5*sqrt(2)/2, 'o');
plot(sqrt(2)/2, 5*sqrt(2)/2, 'o');

error1 = vecnorm(squeeze(simout.p.Data(1:2,1,:))-[5*sqrt(2)/2, sqrt(2)/2]');
error2 = vecnorm(squeeze(simout.p.Data(1:2,1,:))-[-5*sqrt(2)/2, -sqrt(2)/2]');
error3 = vecnorm(squeeze(simout.p.Data(1:2,1,:))-[sqrt(2)/2, 5*sqrt(2)/2]');
error4 = vecnorm(squeeze(simout.p.Data(1:2,1,:))-[-sqrt(2)/2, -5*sqrt(2)/2]');

fprintf("Average distance error from waypoint %f\n",mean([min(error1) min(error2) min(error3) min(error4)]));

%% End-of-file statements

rmdir('slprj', 's')

function X = crossmat(x)
X=[0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end
