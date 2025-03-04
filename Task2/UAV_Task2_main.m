%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A3S EXAM SCRIPT AY 2024/2025
% Author: Giuseppe Anastasio (giuseppe1.anastasio@mail.polimi.it)
% 
% This file contains data for the exam of the A3S course ay 2024/2025.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;

addpath common 

%% SIMULATION SETUP
FLAG_act = 0;
FLAG_sens = 0;
scenario = 2;  % circularTrajectory: 2      setpointTrajectory: 1

% auxiliary parameters
degToRad = pi/180;
radTodeg = 180/pi;
par.g = 9.81; %m/s^2
par.e_1 = [1 0 0]'; par.e_2 = [0 1 0]'; par.e_3 = [0 0 1]';

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
    omegab_0 = [0; 0; 0];
    vb_0 = [0; 0; 0];
    p_0 = [0; 0; 0];
    Omega_0 = [1500 1500 1500 1500]';
    stop_time = '60';

end

%%% Target trajectory for setpoint tracking
targetParams.omega_p_dot = zeros(3,1);
targetParams.v_b_dot = zeros(3,1);
targetParams.omega_p = zeros(3,1); 
targetParams.v_b = zeros(3,1);
targetParams.p = [1 1 -2]';
targetParams.heading = pi/4;


%%% UAV inertial and dynamic parameters

% Nominal values for the parameters
UAV.m = 0.31; %[kg] UAV mass
UAV.mass = UAV.m;
UAV.J = [0.00021     0        0;
               0      0.00024        0;
               0        0      0.00032]; % [kg m^2] inertia matrix
UAV.inertia = UAV.J;
UAV.r_bg = [0 0 0.02]; %[m] center of mass location in the body frame
UAV.S = UAV.m*crossmat(UAV.r_bg) ; % [kg m] static moment
UAV.static = UAV.S;
UAV.M = [UAV.m*eye(3) UAV.S' ; % generalized mass matrix
         UAV.S UAV.J] ;
UAV.Minv = inv(UAV.M) ; % inverse of generalized mass matrix
UAV.Jinv = inv(UAV.J); % inverse of inertial matrix

% Linear Aerodynamics
UAV.D_tauomega = 1*diag([0.00033 0.00022 0.00017]); %  angular velocity damping 
UAV.D_fv =  diag([0.07 0.07 0.1]) ; % linear velocity damping
UAV.D = [UAV.D_fv zeros(3,3); zeros(3,3) UAV.D_tauomega];

%%% Propellers 

UAV.b = 0.08; %[m] arm length (ell in the slides)

UAV.T_max = 3; %[N] max thrust of each propeller
UAV.T_min = 0; %[N] min thrust

UAV.k_m = 70; % [s-1] Inverse of the time constant of the propeller motors
k_f = 0.3379e-6;                % [N/rad^2/s^2] Thrust characteristic coeff
sigma = 5.343e-3;      % [m] Torque-to-thrust ratio

UAV.lambda_r = [1; 1; 1; 1];

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

F2 = [1 1 1 1;
    UAV.b*sin(gamma1) UAV.b*sin(gamma2) UAV.b*sin(gamma3) UAV.b*sin(gamma4);
    -UAV.b*cos(gamma1) -UAV.b*cos(gamma2) -UAV.b*cos(gamma3) -UAV.b*cos(gamma4);
    sigma sigma -sigma -sigma];

F2plus = F2'*pinv(F2*F2');

%%% CONTROL PARAMETERS

%%% SETPOINT TRACKING PARAMETERS RETUNED FOR CIRCULAR TRACKING

% Outer loop
ctrl.KR = diag([15,15,15]); %dcm feedback
ctrl.Kpos = diag([2.9,2.9,2.5]);
% Inner loop
ctrl.Komega = diag([0.002,0.002,0.001]);
ctrl.Kiomega = diag([0.001, 0.001, 0.00007]);
ctrl.Kv = diag([0.48,0.48,0.59]);
ctrl.Kiv = diag([0.014,0.014,0.58]);

ctrl.vMax = 10;

%% SIMULATE MODEL

simout = sim('UAV_Task2.slx','StopTime',stop_time);

%%% PLOT RESULTS

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

%% MC - setpoint

scenario = 1;
popNum = 5;

bgSigma = 1*diag([0;0; 0.005^2]);  % center of gravity
lambdaSigma = 1*diag([0.05^2; 0.05^2; 0.05^2; 0.05^2]); % lambda

r_bg_mat = mvnrnd([0; 0; 0],bgSigma,popNum);
lambda_r_mat = mvnrnd([1; 1; 1; 1],lambdaSigma,popNum);

posOvershoot = zeros(length(r_bg_mat),1);
yawOvershoot = zeros(length(r_bg_mat),1);
pitchOvershoot = zeros(length(r_bg_mat),1);
rollOvershoot = zeros(length(r_bg_mat),1);

posPlot = figure;
nPlot = figure;
ePlot = figure;
dPlot = figure;
yawPlot = figure;
pitchPlot = figure;
rollPlot = figure;


for i=1:length(r_bg_mat)

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
        omegab_0 = [0; 0; 0];
        vb_0 = [0; 0; 0];
        p_0 = [0; 0; 0];
        Omega_0 = [1500 1500 1500 1500]';
        stop_time = '60';
    
    end
    
    % updating center of mass
    UAV.r_bg = r_bg_mat(i,:); %[m] center of mass location in the body frame
    UAV.S = UAV.m*crossmat(UAV.r_bg) ; % [kg m] static moment
    UAV.M = [UAV.m*eye(3) UAV.S'; % generalized mass matrix
             UAV.S UAV.J];
    UAV.Minv = inv(UAV.M); % inverse of generalized mass matrix
    
    % updating propeller effectiveness
    UAV.lambda_r = lambda_r_mat(i,:)';

    % simulation
    simout = sim('UAV_Task2.slx','StopTime',stop_time);

    % evaluating performance parameters
    posOvershoot(i) = max(vecnorm(simout.p.Data(:,:,:))) - norm(simout.p.Data(:,:,end));
    yawOvershoot(i) = rad2deg(max(abs(simout.euler.Data(:,3,:))) - simout.euler.Data(:,3,end));
    pitchOvershoot(i) = rad2deg(max(abs(simout.euler.Data(:,2,:))) - simout.euler.Data(:,2,end));
    rollOvershoot(i) = rad2deg(max(abs(simout.euler.Data(:,1,:))) - simout.euler.Data(:,1,end));

    figure(posPlot)
    hold on
    plot(simout.p.Time,vecnorm(squeeze(simout.p.Data(1:3,:,:))),'LineWidth',1);
    grid minor
    xlabel('[s]')
    ylabel('[m]')
    title('Position')
    legend('Distance')

    figure(nPlot)
    hold on
    plot(simout.p.Time,squeeze(simout.p.Data(1,:,:)),'LineWidth',1);
    grid minor
    xlabel('[s]')
    ylabel('[m]')
    title('Position')
    legend('N')

    figure(ePlot)
    hold on
    plot(simout.p.Time,squeeze(simout.p.Data(2,:,:)),'LineWidth',1);
    grid minor
    xlabel('[s]')
    ylabel('[m]')
    title('Position')
    legend('E')

    figure(dPlot)
    hold on
    plot(simout.p.Time,squeeze(simout.p.Data(3,:,:)),'LineWidth',1);
    grid minor
    xlabel('[s]')
    ylabel('[m]')
    title('Position')
    legend('D')

    figure(yawPlot)
    hold on
    plot(simout.euler.Time,squeeze(simout.euler.Data(1,3,:)) * radTodeg,'LineWidth',1);
    grid minor
    xlabel('[s]')
    ylabel('[deg]')
    title('Attitude [yaw]')
    legend('\psi')

    figure(pitchPlot)
    hold on
    plot(simout.euler.Time,squeeze(simout.euler.Data(1,2,:)) * radTodeg,'LineWidth',1);
    grid minor
    xlabel('[s]')
    ylabel('[deg]')
    title('Attitude [pitch]')
    legend('\theta')

    figure(rollPlot)
    hold on
    plot(simout.euler.Time,squeeze(simout.euler.Data(1,1,:)) * radTodeg,'LineWidth',1);
    grid minor
    xlabel('[s]')
    ylabel('[deg]')
    title('Attitude [roll]')
    legend('\phi')

end

figure
histogram(posOvershoot, popNum);
title("Position overshoot")

figure
histogram(yawOvershoot, popNum);
title("Yaw overshoot")

figure
histogram(pitchOvershoot, popNum);
title("Pitch overshoot")

figure
histogram(rollOvershoot, popNum);
title("Roll overshoot")

%% MC - circle

scenario = 2;
popNum = 5;

bgSigma = 1*diag([0;0; 0.01^2]);
lambdaSigma = 1*diag([0.05^2; 0.05^2; 0.05^2; 0.05^2]);

r_bg_mat = mvnrnd([0; 0; 0],bgSigma,popNum);
lambda_r_mat = mvnrnd([1; 1; 1; 1],lambdaSigma,popNum);

posRMS = zeros(length(r_bg_mat),1);

posPlot = figure;
plot(3*cos(0:0.01:2*pi),3*sin(0:0.01:2*pi),'-.');
grid minor
xlabel('[E]')
ylabel('[N]')
title('Position')
axis equal

for i=1:length(r_bg_mat)

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
        omegab_0 = [0; 0; 0];
        vb_0 = [0; 0; 0];
        p_0 = [0; 0; 0];
        Omega_0 = [1500 1500 1500 1500]';
        stop_time = '60';
    
    end

    UAV.r_bg = r_bg_mat(i,:); %[m] center of mass location in the body frame
    UAV.S = UAV.m*crossmat(UAV.r_bg) ; % [kg m] static moment
    UAV.M = [UAV.m*eye(3) UAV.S'; % generalized mass matrix
             UAV.S UAV.J];
    UAV.Minv = inv(UAV.M); % inverse of generalized mass matrix
    
    UAV.lambda_r = lambda_r_mat(i,:)';

    simout = sim('UAV_Task2.slx','StopTime',stop_time);

    posRMS(i) = mean(simout.posRMS.Data(round(length(simout.posRMS.Data)/2):end));

    figure(posPlot)
    hold on
    plot(squeeze(simout.p.Data(2,:,:)),squeeze(simout.p.Data(1,:,:)),'LineWidth',1);

end

figure
histogram(posRMS, popNum);
title("Position RMS after transient")

%% End-of-file statements

rmdir('slprj', 's')

function X = crossmat(x)
X=[0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end
