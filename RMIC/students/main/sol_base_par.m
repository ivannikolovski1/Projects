%Base parameters --> Students should build the whole function <--
function beta_b =  sol_base_par( Gamma_b, Tau )
    %% Excercise 1
    % This function should solve for the base parameters
    % <<YOUR CODE HERE>>
    Tau_T=Tau'
    for i=1:3
        beta_b= inv(Gamma_b'*Gamma_b)*Gamma_b'*Tau_T(:);
    end
end

