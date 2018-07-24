

%pkg load control
%graphics_toolkit('gnuplot')

%% parameters 

m_wh = 0.058; % MAss of one wheel [kg]

m_mot = 0.2; % Mass of 1 motor stator [Kg]

r = 0.5 * 0.085; % Wheel radius [m]

b = 0.255; % Chassis length (along the axle) [m]

a = 0.1; % Chassis width (perpendicular to the axle) [m]

m_plat = 0.081; % mass of 1 wooden platform [kg]

m_bolt = 0.03; % Mass of 1 bolt [kg]

m_0 = 2*m_mot + 2*m_plat + 4*m_bolt; % total Mass of the body [Kg]

d1 = 0.0325; % distance of the axle from the lower woodden platform [m]

L1 = 0.15; % Length of 1 bolt [m]

L_diag = (a^2 + b^2)^0.5; % Digonal distance of the woodden platform [m]

J_wh = 0.025 ; % Moment of inertia of the wheel and motor rotating parts [kg.m2] obtained by experiment

J_mot_y = 2*J_wh; % It is assumed that the I_y of motor stator = 2 * (I of wheel + gearbox)

J_y = (m_plat*a*a/12 + m_plat*d1^2) + (m_plat*a*a/12 + m_plat*(L1+d1)^2) + 4*((m_bolt*L1^2)/12 + m_bolt*(L1/2+d1)^2 + m_bolt*(a/2)^2) + 2*J_mot_y;

J_z = 2*(m_plat*(a*a + b*b)/12) + 4*(m_bolt * (0.5*L_diag)^2) +2*(m_mot * (0.5*b)^2);

l = (m_plat*d1+4*m_bolt*(0.5*L1+d1)+m_plat*(L1+d1))/m_0; % Distance of vertical CG of the body from wheel centre [m]

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

Q = zeros(4,4); Q(1,1) = 10; Q(2,2) = 1; Q(3,3) = 1; Q(4,4) = 1;

R = 10*[1 0;0 1];

PRM.K = lqr(A,PRM.B_mot_torq,Q,R) 

%% Simulate the robot

t1=0:0.01:100;

IC=[0.10;0.10;0;0]; % defining initial conditions for the robot

function dx = rob_sim(t,y,PRM)
  
  dx = (PRM.A - PRM.B_mot_torq*PRM.K)*y;
  
end
  
[t_out,q_out] = ode45(@(t,y)rob_sim(t,y,PRM),t1,IC);

subplot(2,2,1)
plot(t_out, q_out(:,1)); ylabel('Theta_dot [rad/s]'); xlabel('T [s]');
subplot(2,2,2)
plot(t_out, q_out(:,2)); ylabel('\Theta [rad]'); xlabel('T [s]');
subplot(2,2,3)
plot(t_out, q_out(:,3)); ylabel('V [m/s]'); xlabel('T [s]');
subplot(2,2,4)
plot(t_out, q_out(:,4)); ylabel('\Omega [rad/s]'); xlabel('T [s]');