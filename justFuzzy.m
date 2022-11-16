%% Fuzzy logic based controller for tracking the lissajous trajectory %%
%% Plate angles alpha and beta are the control inputs %%
clc
clear all

%Initializations
time_step=0.01;
time_span=20;
t=0:time_step:time_span;
init_conditions = [-0.03 0.1 -0.02 0.6]';
fis = readfis('PlateAngles.fis'); %Fuzzy Parameters
%System Parameters
K=7;
A=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B=[0 0;K 0;0 0;0 K];
X=init_conditions; %State vector
count=1; %Time instant

for Tinst=0:time_step:time_span

    T(count)=Tinst;
    [desiredxVal,desiredyVal,desiredvxVal,desiredvyVal]= desiredtrajpoint(Tinst);

    %Position and velocity errors
    ErrorX(count) = desiredxVal-X(1);
    ErrorY(count) = desiredyVal-X(3);
    vErrorX(count) = desiredvxVal-X(2);
    vErrorY(count) = desiredvyVal-X(4);
    
    %Fuzzy Logic Controller
    Fuzzy_input=[ErrorX(count), ErrorY(count), vErrorX(count), vErrorY(count)]; %Inputs to Fuzzy Logic controller
    control_input = evalfis(fis,Fuzzy_input); %Control input to the system = output of the Fuzzy Logic controller
    alpha=control_input(1);
    beta=control_input(2);
    inputTraj(count,1)=alpha;
    inputTraj(count,2)=beta;
    
    %Simulate system based on control inputs alpha and beta
    dx=(A*X)+B*[alpha;beta];
    X=X+(dx*time_step);
    Xtraj(count,:)=X;
    desired(1)=desiredxVal;
    desired(2)=desiredyVal;
    desired(3)=desiredvxVal;
    desired(4)=desiredvyVal;
    Dtraj(count,:)=desired; %Storing desired trajectory points
    count=count+1; %Next time instant
end
%[T,x] = ode45(@(t,X) ballplatedynamics(t,X,a,b),tspan,x0);

figure(2)

figure(1)
plot(Xtraj(:,1),Xtraj(:,3),'r-.','LineWidth',2)
hold on
plot(Dtraj(:,1), Dtraj(:,2),'b--','LineWidth',2)
hold on
%axis('square');
title('Trajectory plot')
xlabel('X axis')
h_xlabel = get(gca,'XLabel')
set(h_xlabel,'FontSize',20);
ylabel('Y axis')
h_ylabel = get(gca,'XLabel')
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
legend('Actual Trajectory','Desired Trajectory')
grid on;

figure(2)
plot(T,inputTraj(:,1),'r-.','LineWidth',2)
hold on
plot(T,inputTraj(:,2),'b--','LineWidth',2)
%axis('square');
title('Input Angles vs time')
xlabel('Angles')
h_xlabel = get(gca,'XLabel')
set(h_xlabel,'FontSize',20);
ylabel('Time')
h_ylabel = get(gca,'XLabel')
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
legend('Alpha','Beta')
grid on;

%% Finding the appropriate position and velocity based on the lissajous trajectory %%
function [desiredxVal,desiredyVal,desiredvxVal,desiredvyVal]= desiredtrajpoint(t)
    A=0.04;
    B=0.04;
    a=1;
    b=2;
    delta=pi/2;
    desiredxVal=A*sin((a*t)+delta);
    desiredyVal=B*sin(b*t);
    desiredvxVal=A*a*cos((a*t)+delta);
    desiredvyVal=B*b*cos(b*t);
end