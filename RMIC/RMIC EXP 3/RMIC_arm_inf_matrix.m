% =========================================================================
% Author: Alexander Toedtheide, alexander.toedtheide@tum.de, 2015-02      =
% Editor: Fernando Diaz Ledezma, fernando.diaz@tum.de, 2019-11            =
% (c) Lehrstuhl für Robotik und Systemintelligenz, TUM                    =
% =========================================================================

function F = RMIC_arm_inf_matrix(q, qD, qDD, nop, constPar)
% Function to build the information matrix based on the regressor matrix
% and a viscous and Coulomb friction model                            
                               
%#codegen              
assert(isa(constPar.nop,'double') && isreal(constPar.nop) && all(size(constPar.nop) == [1 1]), ...
  'nop has to be [1x1] double');                                                            
assert(isa(constPar.noj,'double') && isreal(constPar.noj) && all(size(constPar.noj) == [1 1]), ...
  'noj has to be [1x1] double');
assert(isa(constPar.considerFrct,'double') && isreal(constPar.considerFrct) && all(size(constPar.considerFrct) == [1 1]), ...
  'considerFrct has to be [1x1] double');
assert(isa(constPar.considerFrct,'double') && isreal(constPar.considerFrct) && all(size(constPar.considerFrct) == [1 1]), ...
  'considerFrct has to be [1x1] double');
assert(isa(constPar.regIndizes,'double') && isreal(constPar.regIndizes) && all(size(constPar.regIndizes) <= [1 20]));     

%% Initialization
  
    considerFrct = constPar.considerFrct;
    noj          = constPar.noj;
    nrows        = nop*noj;
    ncols        = constPar.regIndizes(2);
    indices      = [-1 0];

    if constPar.considerFrct
        ncols = ncols + 2*noj;
    end
    
    F = NaN(nrows, ncols);

    for i=1:nop
        q_   = q(i,:);
        qD_  = qD(i,:);
        qDD_ = qDD(i,:);

       %% << T2-D >> Function call to get the regressor matrix
        C_tmp = RMIC_regressor_base_matrix(q_,qD_,qDD_,constPar);
        
        % Consideration of friction
        if considerFrct == 1
            C_frct = zeros(noj,noj*2);
            for j = 1:noj
                C_frct(j,1+2*(j-1)) = tanh(constPar.tanh_coef*qD_(1,j)); % sign(qD_(1,j));
                C_frct(j,2+2*(j-1)) = qD_(1,j);
            end
        else
            C_frct = [];
        end

        % Combination of the regressor with the friction model matrix
        % * noj = number of joints
        % * nop = number of points
        
        %% << T2-E >> Construction of the information matrix
        indices      = indices + 2; % <<YOUR CODE HERE>>
        F(indices,:) = [C_tmp, C_frct]; % <<YOUR CODE HERE>>  
    end

    if any(isnan(F))
        error('Information matrix F contains NaN entries!')
    end       
    
end

function C_tmp = RMIC_regressor_base_matrix(q_,qD_,qDD_,constPar)
                 C_tmp = [ qDD_(1), constPar.g*cos(q_(1)), -constPar.g*sin(q_(1)), qDD_(1) + qDD_(2), - sin(q_(2))*qD_(2)^2 - 2*qD_(1)*sin(q_(2))*qD_(2) + constPar.g*cos(q_(1) + q_(2)) + 2*qDD_(1)*cos(q_(2)) + qDD_(2)*cos(q_(2)), - cos(q_(2))*qD_(2)^2 - 2*qD_(1)*cos(q_(2))*qD_(2) - constPar.g*sin(q_(1) + q_(2)) - 2*qDD_(1)*sin(q_(2)) - qDD_(2)*sin(q_(2));
                                0,         0,          0, qDD_(1) + qDD_(2),                                        sin(q_(2))*qD_(1)^2 + constPar.g*cos(q_(1) + q_(2)) + qDD_(1)*cos(q_(2)),                                        cos(q_(2))*qD_(1)^2 - constPar.g*sin(q_(1) + q_(2)) - qDD_(1)*sin(q_(2))];
 
end

