% =========================================================================
% Author: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

function C_b = RMIC_regressor_base_matrix(q,dq,ddq,constPar)

    q1   = q(1);
    q2   = q(2);
    dq1  = dq(1);
    dq2  = dq(2);
    ddq1 = ddq(1);
    ddq2 = ddq(2);
    
    % This is an example of how to use the structure array 'constPar'
    % l1   = constPar.kinParamL2(1);
    % g    = constPar.g;

    C_b= []; % <<YOUR CODE HERE>>

    if(isempty(C_b))
        error('Regressor matrix C_b is empty!')
    end 
    
end
