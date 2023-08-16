function tau_ext = calc_tau_ext(q, params, t)
% global tau_ext_real
tau_ext = zeros(3,1);

switch params.flag_tau_ext
    case 1
%         if t > 1.0 && t < 3
%             
%             f = exp(-(t - 3)^2/(2*(0.1)^2));
%             tau_ext = (10*[-3;2;0]);
%         end
        f = exp(-(t - 3)^2/(2*(0.01)^2));
        tau_ext = 10*[-3;2;0]*f;        
        
    case 2 % External force acting in the end-effector
        if t < 2
            f_ext   = [5;0;0];
            J       = planar3R_Jac(q,params);
            tau_ext = J'*f_ext;
        else
            tau_ext = zeros(3,1);
        end
    otherwise
        tau_ext = zeros(3,1);
end
% tau_ext_real = tau_ext;

end

