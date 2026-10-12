%% Calculating the Fuzzy logic based weight matrix W %%
%% The plate angles (the control inputs): alpha=w1f1+w2f2......, beta=w1'f1'+w2'f2'....%%
%% f1,f2... are calculated using the inputs to the fuzzy controller, and another weight matrix P %%
%% The second weight matrix is adjusted by backpropogating the error %%

function weight = NeuroFuzzy(x, xdot)

    %outputs of the input membership functions
    mu = zeros(5,1);
    phi = zeros(5,1);

    %weight matrix: stores the product of the memebership function values
    w = zeros(25,1);

    %Gaussian Membership Function Centers and Standard Deviations
    for i=1:5
        c=(3.75*i-11.25); 
        s=(2.5);
        mu(i) = exp(-(x-c)^2/(2*s^2));
    end
 
    for i=1:5
        c=(25*i-75);
        s=(10);
        phi(i) = exp(-(xdot-c)^2/(2*s^2));
    end
    
    %Storing values in the weight matrix
    for j=1:5
        for k=1:5
            w(((j-1)*5)+k) = mu(j)*phi(k);
        end
    end

    %Normalizing the values to keep everything in [0,1]
    weight = w/sum(w);
end
 