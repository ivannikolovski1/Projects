%Forward kinematics
function dJac = planar3R_dJac_analytic(t, q, dq, params)
    l1  = params.pkin(1);
    l2  = params.pkin(2);
    l3  = params.pkin(3);
    q1  = q(1);
    q2  = q(2);
    q3  = q(3);
    dq1 = dq(1);
    dq2 = dq(2);
    dq3 = dq(3); 
    
    dJac = [-l1*cos(pi/2+q1)*dq1 - l2*cos(pi/2+q1+q2)*(dq1 + dq2) - l3*cos(pi/2+q1+q2+q3)*(dq1 + dq2 + dq3),-l2*cos(pi/2+q1+q2)*(dq1 + dq2) - l3*cos(pi/2+q1+q2+q3)*(dq1 + dq2 + dq3), -l3*cos(pi/2+q1+q2+q3)*(dq1+dq2+dq3);
            -l1*sin(pi/2+q1)*dq1 - l2*sin(pi/2+q1+q2)*(dq1 + dq2) - l3*sin(pi/2+q1+q2+q3)*(dq1 + dq2 + dq3),-l2*sin(pi/2+q1+q2)*(dq1 + dq2) - l3*sin(pi/2+q1+q2+q3)*(dq1 + dq2 + dq3), -l3*sin(pi/2+q1+q2+q3)*(dq1+dq2+dq3);
                                                                0                                          ,                                      0                                  ,                          0         ];                  
end
