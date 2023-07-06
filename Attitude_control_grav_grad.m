%Attitude control of a spacecraft orbiting earth in a circular orbit
%Considering gravity gradient torque
clear all
close all
I = diag([20,50,10]);
%Linearized matrix A
temp = [zeros(3,3),eye(3)];
r = 7000; %Radius of orbit
n = sqrt(398600/r^3);
A = [temp;...
    ((I(1,1)-I(2,2))*n^2)/I(3,3),0,0,0,0,((I(1,1)-I(2,2)+I(3,3))*n)/I(3,3);...
    0,-(I(1,1)-I(3,3))*3*(n^2)/I(2,2),0,0,0,0;...
    0,0,(I(3,3)-I(2,2))*3*(n^2)/I(1,1),-n,0,0];
B = [zeros(3,3);1/I(3,3),0,0;0,1/I(2,2),0;0,0,I(1,1)];
C = eye(6);
D = 0;

states = {'psi' 'theta' 'phi' 'psidot' 'thetadot' 'phidot'};
inputs = {'Tpsi' 'Ttheta' 'Tphi'};
outputs = {'psi' 'theta' 'phi' 'psidot' 'thetadot' 'phidot'};

sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)

Q = diag([10,10,10,10,10,10]);
R = eye(3);
K =lqr(sys,Q,R);
sys1 = ss(A-B*K,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
%%
figure(1)
step(sys)
figure(2)
step(sys1)
%%Applying to nonlinear control
[t,state] = ode45(@(t,state) grav_nonlinear(t,state,K,I,B),[0 40],[3,-4,5,0,0,0]);

control = zeros(length(t),3);
for i = 1:length(t)
    control(i,:) = -K*state(i,:)';
end

figure(3)
plot(t,state(:,1:3))
legend('Yaw','Pitch','Roll')
xlabel('Time [s]')
ylabel('Angle [Degrees]')
grid on

figure(4)
plot(t,control)
title('Control Torque')
xlabel('Time [s]')
ylabel('Torque [N]')
