
% Run Sawyer_simulink_model

clear variables
clc

%%

% Initial conditions
theta0      = [0;pi/2];
thetadot0   = [0;0];

%%
% Regulation input
thset = [1;pi/2+0.5];

% Input Trajectory
tf          = 5;
tin         = [0:.0001:tf]';

thdes        = [zeros(length(tin),2)];
thdotdes     = [zeros(length(tin),2)];
thddotdes    = [zeros(length(tin),2)];

damping_coeffs = diag([0 0]);

%%
% Input type:   %%%%    1 = regulation          0 = tracking           %%%%
type = 1;

% Control type: %%%%    1 = Joint PD            0 = Inverse dynamics   %%%%
ctltyp = 0;

if ctltyp == 0
    
    % Inverse Dynamics control gains     %%%%    SET THE GAINS HERE    %%%%
    Kp =diag([10 10]);
    Kd =diag([3 3]);
    
else

    % Joint PD control gains     %%%%    SET THE GAINS HERE    %%%%
    Kp =diag([3 10]);
    Kd =diag([1 3]);
    
end


%%
% Simulation parameters
tol     = 1e-6;
maxstep = 0.001;
minstep = 0.0009;

% Run simulation
warning off
sim('Sawyer_simulink_model')
warning on

x_size = size(t,1);
%%
% Plot the response
figure(1)
subplot(2,1,1)
plot(t(1:x_size),reference(1:x_size,1))
hold on
plot(t(1:x_size),theta(1:x_size,1),'k-','linewidth',2)
hold off
ylabel('Position (rad)')
% axis([0 tf 0 reference(end,1)+.5])

subplot(2,1,2)
plot(t(1:x_size),reference(1:x_size,2))
hold on
plot(t(1:x_size),theta(1:x_size,2),'k-','linewidth',2)
hold off
ylabel('Position (rad)')

xlabel('Time (s)')
ylabel('Position (rad)')
legend('x_{des}','x','location','southeast')

figure(2)
subplot(2,1,1)
plot(t(1:x_size),input(1:x_size,1),'k-','linewidth',2)
ylabel('Effort')
subplot(2,1,2)
plot(t(1:x_size),input(1:x_size,2),'k-','linewidth',2)
ylabel('Effort')
xlabel('Time (s)')

figure(3)
subplot(2,1,1)
plot(t(1:x_size),error(1:x_size,1),'k-','linewidth',2)
ylabel('Error')
subplot(2,1,2)
plot(t(1:x_size),error(1:x_size,2),'k-','linewidth',2)
ylabel('Error')
xlabel('Time (s)')

figure(4)
subplot(2,1,1)
plot(t(1:x_size),thetadot(1:x_size,1),'k-','linewidth',2)
ylabel('Thetadot')
subplot(2,1,2)
plot(t(1:x_size),thetadot(1:x_size,2),'k-','linewidth',2)
ylabel('Thetadot')
xlabel('Time (s)')