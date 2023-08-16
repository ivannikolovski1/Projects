%Ploting the robot 
function exc_plot(Time,Q,params,Xd)

close all
%global l1 l2 l3
l1 = params.pkin(1);
l2 = params.pkin(2);
l3 = params.pkin(3);
lth = 0.08;

for i=2:100:length(Time)
    t_s = Time(i)-Time(i-1);
    
    q1   = Q(1,i);
    q2   = Q(2,i);
    q3   = Q(3,i);
    xdx  = Xd(1,i);
    xdy  = Xd(2,i);
    xdth = Xd(3,i);
    
%Cartesian positions of the joints of ROBOT at each time step  
    x1    = [0;0];
    x2    = x1 + [l1*cos(pi/2+q1);l1*sin(pi/2+q1)];
    x3    = x2 + [l2*cos(pi/2+q1+q2);l2*sin(pi/2+q1+q2)];
    xEE   = x3 + [l3*cos(pi/2+q1+q2+q3);l3*sin(pi/2+q1+q2+q3)];
    xEE1r = xEE + [l3*sin(pi/2+q1+q2+q3)/8;-l3*cos(pi/2+q1+q2+q3)/8];
    xEE1l = xEE + [-l3*sin(pi/2+q1+q2+q3)/8;l3*cos(pi/2+q1+q2+q3)/8];
    xEE2r = xEE1r + [l3*cos(pi/2+q1+q2+q3)/8;l3*sin(pi/2+q1+q2+q3)/8];
    xEE2l = xEE1l + [l3*cos(pi/2+q1+q2+q3)/8;l3*sin(pi/2+q1+q2+q3)/8];
    
    
%Plotting the links of ROBOT at each timestep
    plot([x1(1),x2(1)],[x1(2),x2(2)],'b','Linewidth',3.5); hold on;  
    plot([x2(1),x3(1)],[x2(2),x3(2)],'r','Linewidth',3.5);
    plot([x3(1),xEE(1)],[x3(2),xEE(2)],'m','Linewidth',3.5);
    
    plot(x1(1),x1(2),'ko','LineWidth',3,'MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor','K')
    plot(x2(1),x2(2),'ko','LineWidth',3,'MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor','K')
    plot(x3(1),x3(2),'ko','LineWidth',3,'MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor','K')
    
    
    plot([xEE1r(1),xEE1l(1)],[xEE1r(2),xEE1l(2)],'k','Linewidth',3.5);
    plot([xEE1r(1),xEE2r(1)],[xEE1r(2),xEE2r(2)],'k','Linewidth',3.5);
    plot([xEE1l(1),xEE2l(1)],[xEE1l(2),xEE2l(2)],'k','Linewidth',3.5);
    if xdx ~= 0 || xdy ~= 0 
       plot(xdx,xdy,'o','MarkerFaceColor','green');
        if params.controller == 2% xdth ~= 99.999
            plot([xdx+lth*cos(xdth),xdx-lth*cos(xdth)],[xdy+lth*sin(xdth),xdy-lth*sin(xdth)],'g.-','Linewidth',2)
            else
        end    
       else 
    end 
    %title(['time : ' num2str(round(t(i),2),'%4.2f')])
    title(['Time : ', num2str(ceil(Time(i)*10)/10), ' s'])

    hold off 
    axis equal
    xlim([-0.7 0.7])
    ylim([-0.7 0.7])
    xlabel('x [m]','FontSize',15)
    ylabel('y [m]','FontSize',15)
    grid minor;

    pause(0.1)
    
end
