%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A3S EXAM SCRIPT AY 2024/2025
% Author: Giuseppe Anastasio (giuseppe1.anastasio@mail.polimi.it)
% 
% This file contains data for the exam of the A3S course ay 2024/2025.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;

%% SIMULATION INITIALIZATION

trimStr = "analytical"; % DELETE NUMERICAL

% auxiliary variables
degToRad = pi/180 ;
radTodeg = 180/pi ;
Environment.g = 9.81 ; % [m/s^2]
Environment.e_3 = [0 0 1]' ;
trim_evaluation = 0;

%%% Initial conditions
omegab_0 = [0 0 0]' ; %  [rad/s]  angular velocity (body components)
p_0 = [0 0 -2]' ; % [m] position
vb_0 = [0 0 0]' ; % [m/s] linear velocity
R_0 = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1] ;
%disturbance = [cos(pi/16) -sin(pi/16) 0; sin(pi/16) cos(pi/16) 0; 0 0 1] ;
disturbance = eye(3);
q_0 = rotm2quat(disturbance*R_0)';

% UAV inertial and dynamic parameters
% Nominal values for the parameters
UAV.m = 0.31 ; %[kg] UAV mass
UAV.J = [  0.00021   0        0;
           0         0.00024  0;
           0         0        0.00032  ] ; % [kg m^2] inertia matrix
UAV.r_bg = [0 0 0.02]; %[m] center of mass location in the body frame
UAV.S = UAV.m*crossmat(UAV.r_bg) ; % [kg m] static moment
UAV.M = [UAV.m*eye(3) UAV.S' ; % generalized mass matrix
         UAV.S UAV.J] ;
UAV.Minv = inv(UAV.M) ; % inverse of generalized mass matrix

% Linear Aerodynamics
UAV.D_tauomega = diag([0.00033 0.00022 0.00017]) ; %  angular velocity damping 
UAV.D_fv =  diag([0.07 0.07 0.1]) ; % linear velocity damping
UAV.D = [UAV.D_fv zeros(3,3); zeros(3,3) UAV.D_tauomega] ;

% Propellers 
UAV.b = 0.08; %[m] arm length (ell in the slides)
UAV.T_max = 3; %[N] max thrust of each propeller
UAV.T_min = 0; %[N] min thrust
UAV.k_m = 70; % [s-1] Inverse of the time constant of the propeller motors
k_f = 0.3379e-6;                % [N/rad^2/s^2] Thrust characteristic coeff
sigma = 5.343e-3;      % [m] Torque-to-thrust ratio
UAV.lambda_r = 1;

% Control allocation
UAV.n = [0; 0; 1] ;
p1 = UAV.b * [ sqrt(2)/2;  sqrt(2)/2;  0] ;
p2 = UAV.b * [-sqrt(2)/2; -sqrt(2)/2;  0] ;
p3 = UAV.b * [ sqrt(2)/2; -sqrt(2)/2;  0] ;
p4 = UAV.b * [-sqrt(2)/2;  sqrt(2)/2;  0] ;
UAV.F = -[UAV.n UAV.n UAV.n UAV.n;...
    (crossmat(p1)-sigma*eye(3))*UAV.n (crossmat(p2)-sigma*eye(3))*UAV.n...
    (crossmat(p3)+sigma*eye(3))*UAV.n (crossmat(p4)+sigma*eye(3))*UAV.n] ;
UAV.Finv = pinv(UAV.F) ;

% epsilon = [-1 -1 1 1]';
% UAV.p1 = UAV.b*[cos(pi/4); sin(pi/4); 0];
% UAV.p2 = UAV.b*[-cos(pi/4); -sin(pi/4); 0];
% UAV.p3 = UAV.b*[cos(pi/4); -sin(pi/4); 0];
% UAV.p4 = UAV.b*[-cos(pi/4); sin(pi/4); 0];
% par.e_3 = Environment.e_3;
% UAV.Fstar = -[par.e_3 par.e_3 par.e_3 par.e_3;
%     (crossmat(UAV.p1)+epsilon(1)*sigma*eye(3))*par.e_3 (crossmat(UAV.p2)+epsilon(2)*sigma*eye(3))*par.e_3 (crossmat(UAV.p3)+epsilon(3)*sigma*eye(3))*par.e_3 (crossmat(UAV.p4)+epsilon(4)*sigma*eye(3))*par.e_3];


% Commands (Open loop: 1 - Thrust, 2 - Omega)
Command_type = 2 ;
T_cmd = [0.760275 0.760275 0.760275 0.760275]' ;
Omega_cmd = [1600 1600 1600 1600]';     % tentative Omega
Omega_0 = Omega_cmd;

% Task 1.2 - Stabilize UAV at target position
omegab_d = [0 0 0]' ; %  [rad/s]  angular velocity (body components)
p_d = [0 0 -2]' ; % [m] position
vb_d = [0 0 0]' ; % [m/s] linear velocity
R_d = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1] ;
q_d = rotm2quat(R_d)' ;
% Kpx = 10 ;
% Kpy = 10 ;
% Kpz = 10 ;
% Kdx = 10 ;
% Kdy = 10 ;
% Kdz = 10 ;

%% TRIM EVALUATION

if trimStr == "numerical"
    %%% Numerical computation of trim
    trim_evaluation = 1;  % allows the trim function to inject its 'u'.
    y = [p_d; omegab_d; vb_d; q_d]; % target output
    u = [1550 1550 1550 1550]';     % tentative trim (input 'u')
    x = [];
    ix = [];
    iu = [];
    iy = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]; % index target all output elements
    [x,u,y,dx,options] = trim('UAV_Task_1',x,u,y,ix,iu,iy);
    Omega_cmd = u;
    Omega_0 = Omega_cmd;
    trim_evaluation = 0;
elseif trimStr == "analytical"
    %%% Analytical computation of trim
    e_3 = Environment.e_3;
    g = Environment.g;
    nub_d = [vb_d; omegab_d];
    cross_six = [crossmat(nub_d(4:6)), zeros(3,3);
                 crossmat(nub_d(1:3)), crossmat(nub_d(4:6))];
    % Steady state dynamics
    w_g = g*[UAV.m*R_d'*e_3; UAV.S*R_d'*e_3];
    u = UAV.Finv*(cross_six*UAV.M*nub_d + UAV.D*nub_d - w_g);
    
    trimedOmega = sqrt(u/k_f);
    Omega_cmd = mean(trimedOmega)*ones(4,1);  % mean to minimize numerical artifacts
    Omega_0 = Omega_cmd;
end

%% SIMULATION RUN

stop_time = 100;
simout = sim('UAV_Task_1.slx','StopTime','stop_time');

%%% RESULTS PLOTS

figure
plot(simout.euler * radTodeg,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[deg]')
title('Attitude')
legend('\phi', '\theta', '\psi')

figure
plot(simout.omegab,'LineWidth',3);
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
title('Position NED frame')
legend('x', 'y', 'z')

figure
plot(simout.vb,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[m/s]')
title('Linear body velocity')
legend('v_x', 'v_y', 'v_z')

function x_cross = crossmat(x)

    x_cross = [0 -x(3) x(2);
               x(3) 0 -x(1);
              -x(2) x(1) 0];

end
