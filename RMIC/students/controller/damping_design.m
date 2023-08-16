%Torque input for the robot
function D_x = damping_design(K_x, MK)
    damping_factor = 0.1;
    [Vc, Lambda]   = eig(K_x, MK);
    lambda         = real(diag(Lambda));
    V              = real(Vc);
    Vinv           = pinv(V);
    lambda2        = sqrt(lambda)*2*damping_factor;
    D_x            = MK*V*diag(lambda2)*Vinv;
end

