syms x1 x2 x3 x4 u1 u2;
% Initialisation of parameters
m1=1.4;m2=1.1;l1=0.3;l2=0.33;s1=.11;s2=0.16;I1=0.025;I2=0.045;
d1=I1+I2+m2*l1^2; d2=m2*l1*s2; d3=I2;
dt=0.01; % Step Time
switch target_num
    case 1
        % Target Position
        th1=35*pi/180;th2=75*pi/180;
        % Initial Position
        th11=40*pi/180;th22=130*pi/180;
    case 2
        th1=70.75*pi/180;th2=55.27*pi/180;
        th11=40*pi/180;th22=130*pi/180;
    case 3
        th1=93.23*pi/180;th2=57.65*pi/180;
        th11=40*pi/180;th22=130*pi/180;
    case 4
        th1=103.07*pi/180;th2=84.25*pi/180;
        th11=40*pi/180;th22=130*pi/180;
end
% Non-linear model - Xdot=F+G*U;
M=[d1+2*d2*cos(x2+th2) d3+d2*cos(x2+th2);d3+d2*cos(x2+th2) d3]; % Inertia matrix
C=d2*sin(x2+th2)*[-x4*(2*x3+x4);x3^2];% Centripetal and coriolis forces matrix
B=[0.05 0.025;0.025 0.05]; % Joint friction matrix
F=[x3;x4;inv(M)*(-C-B*[x3;x4])];
G=[0 0;0 0;inv(M)];
U=[u1;u2];
X_dot=F+G*U;
%Linearised model
%A = dF/dx and B= dF/du=G
A=[0 0 1 0;0 0 0 1;0 0 diff(X_dot(3),x3) diff(X_dot(3),x4);0 0 diff(X_dot(4),x3) diff(X_dot(4),x4)];
B=G;
t=0:dt:0.6; %simulation time
ut=zeros(2,length(t)-1); % control vector
X=zeros(4,length(t)); % true states
Xin=[th11-th1;th22-th2;0;0]; % starting state
% Cost function matrices
Q=[1 0 0 0;0 10 0 0;0 0 1 0;0 0 0 1];
Qf=[10000 0 0 0;0 10000 0 0;0 0 1000 0;0 0 0 1000];
R=eye(2);

