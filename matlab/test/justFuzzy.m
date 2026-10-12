clc
clear all
tstep=0.01;
tspan=20;
t=0:tstep:tspan;
x0 = [0.0 0.0 -0.00 0.0]';
fis = readfis('PlateAngles.fis');
K=7;
A=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B=[0 0;K 0;0 0;0 K];
count=1;
X=x0;

loss=0;

for Tinst=0:0.01:tspan
    T(count)=Tinst;
    [desiredxVal,desiredyVal,desiredvxVal,desiredvyVal]= desiredtrajpoint(Tinst);
    ErrorX(count) = desiredxVal-X(1);
    ErrorY(count) = desiredyVal-X(3);
    vErrorX(count) = desiredvxVal-X(2);
    vErrorY(count) = desiredvyVal-X(4);
    loss_p(count)=0.5*(ErrorX(count))^2+0.5*(ErrorY(count))^2;
    loss_v(count)=0.5*(vErrorX(count))^2+0.5*(vErrorY(count))^2;
    input = evalfis(fis,[ErrorX(count), ErrorY(count), vErrorX(count), vErrorY(count)]);
    alpha=input(1);
    beta=input(2);
    inputTraj(count,1)=alpha;
    inputTraj(count,2)=beta;
    dx=(A*X)+B*[alpha;beta];
    X=X+(dx*tstep);
    Xtraj(count,:)=X;
    desired(1)=desiredxVal;
    desired(2)=desiredyVal;
    desired(3)=desiredvxVal;
    desired(4)=desiredvyVal;
    Dtraj(count,:)=desired;
    count=count+1;
end
%[T,x] = ode45(@(t,X) ballplatedynamics(t,X,a,b),tspan,x0);
rmse_pos=sqrt(sum(loss_p)/length(loss_p))
rmse_vel=sqrt(sum(loss_v)/length(loss_v))
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

function [desiredxVal,desiredyVal,desiredvxVal,desiredvyVal]= desiredtrajpoint(t)
    A=0.04;
    B=0.04;
    a=0.5;
    b=1;
    delta=pi/2;
%     desiredxVal=A*sin((a*t)+delta);
%     desiredyVal=B*sin(b*t);
%     desiredvxVal=A*a*cos((a*t)+delta);
%     desiredvyVal=B*b*cos(b*t);
    desiredxVal=A*cos(t)*(1-cos(t));
    desiredyVal=B*sin(t)*(1-cos(t));
    desiredvxVal=A*sin(t)*(2*cos(t) -1);
    desiredvyVal=B*((sin(t))^2-(cos(t))^2 + cos(t));
end
function dx = ballplatedynamics(t,X,a,b)
    K=7;
    A=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
    B=[0 0;K 0;0 0;0 K];
    dx=(A*X)+B*[a;b];
    % d_x1=dx(1);
    % d_x2=dx(2);
    % d_x3=dx(3);
    % d_x4=dx(4);
end