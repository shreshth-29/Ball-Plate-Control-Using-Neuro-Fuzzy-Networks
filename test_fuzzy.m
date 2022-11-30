%% Testing phase, simulating the system using the values of P obtained after training %%
clc;
clear all;

%Initializations
time_step=0.01;
time_span=100;
t=0:time_step:time_span;
init_conditions_train = [0.01 0.04 0.03 -0.06]'; %Initial conditions for training
init_conditions_test = [0.0 0.00 0.00 -0.00]'; %Initial conditions for testing
count=1; %Time instant
Xsys=init_conditions_test;
learning_rate=0.7;

%System parameters 
K=7;
A=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B=[0 0;K 0;0 0;0 K];

%Getting the appropriate weight matrices after training
[p1,p2]=train_fuzzy(learning_rate,time_step,10*time_span,A,B,init_conditions_train)

%% Testing %%

for Tinst=0:time_step:time_span
   
    T(count)=Tinst;
    [desiredxVal,desiredyVal,desiredvxVal,desiredvyVal]= desiredtrajpoint(Tinst); %Desired position and velocity
    
    %Position and velocity errors
    ErrorX(count) = desiredxVal-Xsys(1);
    ErrorY(count) = desiredyVal-Xsys(3);
    vErrorX(count) = desiredvxVal-Xsys(2);
    vErrorY(count) = desiredvyVal-Xsys(4);

    loss_p(count)=0.5*(ErrorX(count))^2+0.5*(ErrorY(count))^2;
    loss_v(count)=0.5*(vErrorX(count))^2+0.5*(vErrorY(count))^2;

    %Calculate alpha
    X=[1;ErrorX(count); vErrorX(count)];
    F1=p1*X;
    W1=NeuroFuzzy(ErrorX(count),vErrorX(count));
    z1=W1'*F1;
    alpha=tansig(z1);
    
    %Calculate beta
    Y=[1;ErrorY(count); vErrorY(count)];
    F2=p2*Y;
    W2=NeuroFuzzy(ErrorY(count),vErrorY(count));
    z2=W2'*F2;
    beta=tansig(z2);

    %Update system based on calculated alpha and beta
    dx=(A*Xsys)+B*[alpha;beta];
    Xsys=Xsys+(dx*time_step);

    Xtraj(count,:)=Xsys;
    desired(1)=desiredxVal;
    desired(2)=desiredyVal;
    desired(3)=desiredvxVal;
    desired(4)=desiredvyVal;

    Dtraj(count,:)=desired; %Storing the desired trajectory points
    input(count , :)=[alpha,beta]; %storing control input values, i.e alpha,beta
    count=count+1;
    
end

rmse_p=sqrt(sum(loss_p)/length(loss_p))
rmse_p=sqrt(sum(loss_v)/length(loss_v))

figure(1) %Desired and actual trajectory vs time
plot(Xtraj(:,1),Xtraj(:,3),'r--','LineWidth',2)
hold on
plot(Dtraj(:,1), Dtraj(:,2),'b-.','LineWidth',2)
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

% figure(2) %Control inputs (alpha and beta) vs time
% plot(1:count-1,input(:,2),'b-','LineWidth',2)
% hold on
% plot(1:count-1,input(:,1),'r-.','LineWidth',2)
% %axis('square');
% title('Input Angles vs time')
% xlabel('Angles')
% h_xlabel = get(gca,'XLabel')
% set(h_xlabel,'FontSize',20);
% ylabel('Time')
% h_ylabel = get(gca,'XLabel')
% set(h_ylabel,'FontSize',20);
% set(gca,'FontSize',12)
% legend('Beta','Alpha')
% grid on;


%% Finding the appropriate position and velocity based on the lissajous trajectory %%
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

