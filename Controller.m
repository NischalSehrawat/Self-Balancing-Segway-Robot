

%pkg load control
%graphics_toolkit('gnuplot')

clear all; close all;

%% parameters 

t_sim = 5; % Total simulation time [s]

m_wh = 0.058 + 0.2; % MAss of one wheel + mass of 1 motor [kg]

r = 0.5 * 0.085; % Wheel radius [m]

b = 0.255; % Chassis length (along the axle) [m]

a = 0.1; % Chassis width (perpendicular to the axle) [m]

m_plat = 0.081; % mass of 1 wooden platform [kg]

m_bolt = 0.03; % Mass of 1 bolt [kg]

m_0 = 2*m_plat + 4*m_bolt; % total Mass of the body / chassis (excluding wheel and motors) [Kg]

d1 = 0.0325; % distance of the axle from the lower woodden platform [m]

L1 = 0.15; % Length of 1 bolt [m]

L_diag = (a^2 + b^2)^0.5; % Digonal distance of the woodden platform [m]

J_y = 2*(m_plat*a*a/12 + m_plat*(0.5*L1)^2) + 4*(m_bolt*(L1^2)/12 + m_bolt*(0.5*a)^2);

J_z = 2*(m_plat*(a*a + b*b)/12) + 4*(m_bolt * (0.5*L_diag)^2);

J_wh = 0.5*m_wh * r^2; % Moment of inertia of wheel + motor through its axis i.e. 0.5*mr^2 [kgm2] 

l = (m_plat*d1+4*m_bolt*(0.5*L1+d1)+m_plat*(L1+d1))/m_0 ; % Distance of vertical CG of the body from axle [m]

g = 9.8; % Acceleration due to gravity [m/s2]

J5 = J_z + 0.75*m_wh*b^2 + 0.5*m_wh*r*r; % Moment of inertia

M = 2*m_0*m_wh*l*l*r*r + 2*J_wh*m_0*l^2 +  J_y*m_0*r^2 + 2*J_y*m_wh*r^2 + 2*J_wh*J_y;

%% A and B matrices

A = zeros(4,4); B = zeros(4,2);

A(2,1) = 1 ; 
A(1, 2) = (m_0*r*r+2*m_wh*r*r+2*J_wh)*m_0*l*g / M;
A(3,2) = -m_0*m_0*l*l*r*r*g / M;

B(1,1) = -(l*m_0*r+m_0*r*r+2*m_wh*r*r+2*J_wh)/M ; % motor torques in terms of sum and difference of motor torques
B(3,1) = r*(l*l*m_0+l*m_0*r+J_y)/M; 
B(4,2) = b/(2*r*J5);

PRM.B_mot_torq = B*[1 1;1 -1] ;% Individual motor torques

PRM.A= A;

Q = zeros(4,4); 
Q(1,1) = 10; % For Angular velocity 
Q(2,2) = 10; % For Angle
Q(3,3) = 1; % For Linear velocity
Q(4,4) = 1; % For turning velocity


R = 1000*[1 0;0 1];

PRM.K = lqr(A,PRM.B_mot_torq,Q,R);

lw = 1;

%% Simulate the robot

t1=0:0.01:t_sim;

IC=[0.10;0.10;0;0]; % defining initial conditions for the robot
 
[t_out,q_out] = ode45(@(t,y)rob_sim(t,y,PRM),t1,IC);

subplot(2,2,1)
plot(t_out, q_out(:,1),'LineWidth',lw); xlabel('T [s]');
hold on;
hl = ylabel('$\dot{\Theta} [rad/s]$');
set(hl, 'Interpreter', 'latex');
%%
subplot(2,2,2)
plot(t_out, q_out(:,2),'LineWidth',lw); ylabel('\Theta [rad]'); xlabel('T [s]');
hold on;

subplot(2,2,3)
plot(t_out, q_out(:,3),'LineWidth',lw); ylabel('V [m/s]'); xlabel('T [s]');
hold on;

subplot(2,2,4)
pp = q_out(:,4);
pp(abs(pp)<1e-3)=0 ;
plot(t_out, pp,'LineWidth',lw); ylabel('\Omega [rad/s]'); xlabel('T [s]');
hold on;

outputs = q_out*PRM.K';

figure

plot(t_out, outputs,'LineWidth',lw);

xlabel('T [s]'); ylabel('Torque [Nm]'); 