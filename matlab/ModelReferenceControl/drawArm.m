function drawArm(u,L,w)

    % process inputs to function
    theta    = u(1);
    t        = u(2);
    
    % define persistent variables 
    persistent arm_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        plot([0,L],[0,0],'k--'); % plot track
        hold on
        arm_handle = drawArm_(theta, L, w, [], 'normal');
        axis([-2*L, 2*L, -2*L, 2*L]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawArm_(theta, L, w, arm_handle);
    end
end

   
%
%=======================================================================
% drawArm_
% draw the arm
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawArm_(theta, L, w, handle, mode)
  
  pts = [...
      0, -w/2;...
      L, -w/2;...
      L, w/2;...
      0, w/2;...
      ]';
  pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = fill(X,Y,'b', 'EraseMode', mode);
    %handle = plot(X,Y,'b', 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 