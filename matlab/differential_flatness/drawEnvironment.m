% Differential flatness example for ECE 774
%
% Modified: 4/15/2014 - R. Beard
%
%
function drawEnvironment(uu,P)

    rx = uu(1);
    ry = uu(2);
    psi = uu(3);
    V   = uu(4);
    rx_r = uu(5);
    ry_r = uu(6);
    psi_r = uu(7);
    V_r   = uu(8);
    t   = uu(9);
    
    % define persistent variables 
    persistent rob_handle;  % figure handle for home team
    persistent ref_handle;  % figure handle for home team
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        ref_handle = drawRobot(rx_r,ry_r,psi_r,[],'g','normal',P);
        hold on
        rob_handle = drawRobot(rx,ry,psi,[],'b','normal',P);
        axis([-5,5,-5,5]);
    else
        drawRobot(rx_r,ry_r,psi_r,ref_handle,'g','normal',P);
        drawRobot(rx,ry,psi,rob_handle,'b','normal',P);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawRobot(rx,ry,psi,handle,color,mode,P)
    th = [0, -pi/8, -pi/4, -3*pi/8, -pi/2, -5*pi/8, -3*pi/4, -7*pi/8, -pi, 0];
    pts = 0.5*[sin(th); cos(th)];
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    pts = R*pts;
    pts = pts + repmat([rx; ry],1,size(pts,2));

    if isempty(handle),
        handle = fill(pts(1,:), pts(2,:),color,'EraseMode', mode);
    else
        set(handle,'XData',pts(1,:),'YData',pts(2,:));
        drawnow
    end
  
end 


  

  