%% Training phase for finding the weight matrices P for alpha and beta %%

function[p1,p2]=train_fuzzy(learning_rate,time_step,time_span,A,B,init_condition)

    rand('seed',1) %setting the seed

    %Randomly initializing the weight matrices
    p2=rand(25,3);
    p1=rand(25,3);

    t=0:time_step:time_span;
    count=1; %time instants
    Xsys=init_condition;

    dw_prev_1=0;
    dw_curr_1=0;
    dw_prev_2=0;
    dw_curr_2=0;

    gamma=0.9;
    
    for Tinst=0:time_step:time_span
        T(count)=Tinst;
        [desiredxVal,desiredyVal,desiredvxVal,desiredvyVal]= desiredtrajpoint(Tinst);
        desiredAlpha=desiredxVal/7;
        desiredBeta=4*desiredyVal/7;
        
        %Position and velocity errors
        ErrorX(count) = desiredxVal-Xsys(1);
        ErrorY(count) = desiredyVal-Xsys(3);
        vErrorX(count) = desiredvxVal-Xsys(2);
        vErrorY(count) = desiredvyVal-Xsys(4);

        DesiredAlphaTraj(count)=desiredAlpha;
        DesiredBetaTraj(count)=desiredBeta;
        
        %Control in the x direction
        X=[1;ErrorX(count); vErrorX(count)]; %input to fuzzy logic controller
        F1=p1*X;
        W1=NeuroFuzzy(ErrorX(count),vErrorX(count)); %weights based on the fuzzy logic controller
        alpha=W1'*F1;

        %constants for backpropogation
        a1=0.01;
        b1=0.02;
        lam1=a1*ErrorX(count)+(b1)*vErrorX(count);
        for i=1:25
            for j=1:3
                if j==1
                    dp1=W1(i)*lam1*1;
                elseif j==2
                    dp1=W1(i)*lam1*ErrorX(count);
                else 
                    dp1=W1(i)*lam1*vErrorX(count);
                end
                %Weight update using gradient descent, with gradient
                %estimated from backpropogation
                dw_curr_1=learning_rate*dp1;
                p1(i,j)=p1(i,j)+ (gamma)*dw_curr_1+ (1-gamma)*dw_prev_1;
                dw_prev_1=dw_curr_1;
            end
        end
    
        %Control in the y direction
        Y=[1;ErrorY(count); vErrorY(count)];
        F2=p2*Y;
        W2=NeuroFuzzy(ErrorY(count),vErrorY(count));
        beta=W2'*F2;

        %update weights
        a2=0.01;
        b2=0.02;
        lam2=(a2*ErrorY(count))+((b2)*vErrorY(count));
        for i=1:25
            for j=1:3
                if j==1
                    dp2=W2(i)*lam2*1;
                elseif j==2
                    dp2=W2(i)*lam2*ErrorY(count);
                else 
                    dp2=W2(i)*lam2*vErrorY(count);
                end
                %Weight update using gradient descent, with gradient
                %estimated from backpropogation
                dw_curr_2=learning_rate*dp2;
                p2(i,j)=p2(i,j)+ (gamma)*dw_curr_2+ (1-gamma)*dw_prev_2;
                dw_prev_2=dw_curr_2;
            end
        end

        %Update system based on newly calculated alpha and beta
        dx=(A*Xsys)+B*[alpha;beta];
        Xsys=Xsys+(dx*time_step);
        Xtraj(count,:)=Xsys;

        desired(1)=desiredxVal;
        desired(2)=desiredyVal;
        desired(3)=desiredvxVal;
        desired(4)=desiredvyVal;
        Dtraj(count,:)=desired;
        
        %go to next time instant
        count=count+1;
    end
    
end


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