function handle = draw_waypoints(fig_handle, wpt_list, radius, handle, mode)
% DRAW_WAYPOINTS - plot waypoints

    if wpt_list(1,4)==-9999 % check to see if Dubins paths
        XX = [wpt_list(:,1)];
        YY = [wpt_list(:,2)];
        ZZ = [wpt_list(:,3)];
    else
        XX = [];
        YY = [];
        ZZ = [];
        for i=2:size(wpt_list,1)
            dubinspath = compute_dubins_param(wpt_list(i-1,:),wpt_list(i,:),radius);
            [tmpX,tmpY,tmpZ] = points_along_dubins_path(dubinspath,0.1);
            XX = [XX; tmpX];
            YY = [YY; tmpY];  
            ZZ = [ZZ; tmpZ];
        end
%         ZZ = wpt_list(i,3)*ones(size(XX));
    end
    
    if isempty(handle)
        handle = plot3(fig_handle.Children, YY, XX, -ZZ, 'b');
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,Y,Z] = points_along_dubins_path(dubinspath,Del)
% Points Along Dubins Path -
%   Find points along Dubin's path separted by Del (to be used in
%   collision detection)

  % points along start circle
  th1 = mod(atan2(dubinspath.ps(2)-dubinspath.cs(2),dubinspath.ps(1)-dubinspath.cs(1)),2*pi);
  th2 = mod(atan2(dubinspath.w1(2)-dubinspath.cs(2),dubinspath.w1(1)-dubinspath.cs(1)),2*pi);
  if dubinspath.lams>0
      if th1>=th2
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  X = [];
  Y = [];
  Z = [];
  for i=1:length(th)
    X = [X; dubinspath.cs(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.cs(2)+dubinspath.R*sin(th(i))];
    Z = [Z; dubinspath.cs(3)];
  end
  
  % points along straight line 
  sig = 0;
  while sig<=1
      X = [X; (1-sig)*dubinspath.w1(1) + sig*dubinspath.w2(1)];
      Y = [Y; (1-sig)*dubinspath.w1(2) + sig*dubinspath.w2(2)];
      Z = [Z; (1-sig)*dubinspath.w1(3) + sig*dubinspath.w2(3)];
      sig = sig + Del;
  end
    
  % points along end circle
  th2 = mod(atan2(dubinspath.pe(2)-dubinspath.ce(2),dubinspath.pe(1)-dubinspath.ce(1)),2*pi);
  th1 = mod(atan2(dubinspath.w2(2)-dubinspath.ce(2),dubinspath.w2(1)-dubinspath.ce(1)),2*pi);
  if dubinspath.lame>0
      if th1>=th2
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  for i=1:length(th)
    X = [X; dubinspath.ce(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.ce(2)+dubinspath.R*sin(th(i))];
    Z = [Z; dubinspath.ce(3)];
  end
end