%% Testing phase, simulating the system using the values of P obtained after training %%
clc;
clear all;

%Initializations
time_step=0.01;
time_span=100;
t=0:time_step:time_span;
init_conditions_train = [0.01 0.04 0.03 -0.06]'; %Initial conditions for training
init_conditions_test = [0.04 0.00 0.00 -0.00]'; %Initial conditions for testing
count=1; %Time instant
Xsys=init_conditions_test;
learning_rate=0.15;

%System parameters 
K=7;
A=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B=[0 0;K 0;0 0;0 K];

%Getting the appropriate weight matrices after training
[p1,p2]=train_fuzzy(learning_rate,time_step,time_span,A,B,init_conditions_train)

%% Testing %%

for Tinst=0:time_step:time_span
   
    T(count)=Tinst;
    [desiredxVal,desiredyVal,desiredvxVal,desiredvyVal]= desiredtrajpoint(Tinst); %Desired position and velocity
    
    %Position and velocity errors
    ErrorX(count) = desiredxVal-Xsys(1);
    ErrorY(count) = desiredyVal-Xsys(3);
    vErrorX(count) = desiredvxVal-Xsys(2);
    vErrorY(count) = desiredvyVal-Xsys(4);

    %Calculate alpha
    X=[1;ErrorX(count); vErrorX(count)];
    F1=p1*X;
    W1=NeuroFuzzy(ErrorX(count),vErrorX(count));
    alpha=W1'*F1;
    
    %Calculate beta
    Y=[1;ErrorY(count); vErrorY(count)];
    F2=p2*Y;
    W2=NeuroFuzzy(ErrorY(count),vErrorY(count));
    beta=W2'*F2;

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

figure(1) %Desired and actual trajectory vs time
hold on
plot(Xtraj(:,1),Xtraj(:,3))
plot(Dtraj(:,1),Dtraj(:,2))

figure(2) %Control inputs (alpha and beta) vs time
plot(1:count-1,input(:,1),1:count-1,input(:,2))


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

