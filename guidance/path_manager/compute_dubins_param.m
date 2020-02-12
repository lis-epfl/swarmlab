function dubinspath = compute_dubins_param(start_node, end_node, R)

% COMPUTE_DUBINS_PARAM - Find Dubin's parameters between two configurations
%
% Inputs:
%   start_node  - [wn_s, we_s, wd_s, chi_s, 0, 0]
%   end_node    - [wn_e, wn_e, wd_e, chi_e, 0, 0]
%   R           - minimum turn radius
%
% Outputs:
%   dubinspath          - a matlab structure with the following fields
%       dubinspath.ps   - the start position in re^3
%       dubinspath.chis - the start course angle
%       dubinspath.pe   - the end position in re^3
%       dubinspath.chie - the end course angle
%       dubinspath.R    - turn radius
%       dubinspath.L    - length of the Dubins path
%       dubinspath.cs   - center of the start circle
%       dubinspath.lams - direction of the start circle
%       dubinspath.ce   - center of the end circle
%       dubinspath.lame - direction of the end circle
%       dubinspath.w1   - vector in re^3 defining half plane H1
%       dubinspath.q1   - unit vector in re^3 along straight line path
%       dubinspath.w2   - vector in re^3 defining position of half plane H2
%       dubinspath.w3   - vector in re^3 defining position of half plane H3
%       dubinspath.q3   - unit vector defining direction of half plane H3
% 

  line_length = norm(start_node(1:2)-end_node(1:2));
  if line_length < 2*R
      disp('The distance between nodes must be larger than 2R.');
      dubinspath = [];
  else
    
    ps   = start_node(1:3)';
    chis = start_node(4);
    pe   = end_node(1:3)';
    chie = end_node(4);

    crs = ps + R*Rz(pi/2)*[cos(chis); sin(chis); 0];
    cls = ps + R*Rz(-pi/2)*[cos(chis); sin(chis); 0];
    cre = pe + R*Rz(pi/2)*[cos(chie); sin(chie); 0];
    cle = pe + R*Rz(-pi/2)*[cos(chie); sin(chie); 0];
   
    % compute L1
    line_length = norm(crs-cre);
    theta = atan((cre(2)-crs(2))/(cre(1)-crs(1)));
    L1 = line_length + ...
        R*wrapTo2Pi(2*pi + wrapTo2Pi(theta - pi/2) - wrapTo2Pi(chis - pi/2)) + ...
        R*wrapTo2Pi(2*pi + wrapTo2Pi(chie - pi/2) - wrapTo2Pi(theta - pi/2));
    % compute L2
    line_length = norm(cle-crs);
    theta = pi - atan((cle(2)-crs(2))/(cle(1)-crs(1)));
    theta2 = theta - pi/2 + asin(2*R/line_length);
    if isreal(theta2)==0
      L2 = 9999;
    else
      L2 = sqrt(line_length^2 - 4*R^2) + ...
          R*wrapTo2Pi( 2*pi + wrapTo2Pi(theta2) - wrapTo2Pi(chis-pi/2)) + ...
          R*wrapTo2Pi( 2*pi + wrapTo2Pi(theta2+pi) - wrapTo2Pi(chie+pi/2));
    end
    % compute L3
    line_length = norm(cre-cls);
    theta = atan((cre(2)-cls(2))/(cre(1)-cls(1)));
    theta2 = acos(2*R/line_length);
    if isreal(theta2)==0
      L3 = 9999;
    else
      L3 = sqrt(line_length^2 - 4*R^2) + ...
          R*wrapTo2Pi( 2*pi + wrapTo2Pi(chis+pi/2) - wrapTo2Pi(theta+theta2)) + ...
          R*wrapTo2Pi( 2*pi + wrapTo2Pi(chie-pi/2) - wrapTo2Pi(theta+theta2-pi));
    end
    % compute L4
    line_length = norm(cls - cle);
    theta = atan((cre(2)-cls(2))/(cre(1)-cls(1)));
    L4 = line_length + ...
          R*wrapTo2Pi( 2*pi + wrapTo2Pi(chis+pi/2) - wrapTo2Pi(theta+pi/2)) + ...
          R*wrapTo2Pi( 2*pi + wrapTo2Pi(theta+pi/2) - wrapTo2Pi(chie+pi/2));
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    e1 = [1; 0; 0];
    switch(idx)
        case 1
            cs = crs;
            lams = 1;
            ce = cre;
            lame = 1;
            q1 = (ce-cs)/norm(ce-cs);
            w1 = cs + R*Rz(-pi/2)*q1;
            w2 = ce + R*Rz(-pi/2)*q1;
        case 2   
            cs = crs;
            lams = 1;
            ce = cle;
            lame = -1;
            line_length = norm(ce-cs);
            theta = atan2((ce(2)-cs(2)),(ce(1)-cs(1)));
            theta2 = theta - pi/2 + asin(2*R/line_length);
            q1 = Rz(theta2 + pi/2)*e1;
            w1 = cs + R*Rz(theta2)*e1;
            w2 = ce + R*Rz(theta2 + pi)*e1;
        case 3
            cs = cls;
            lams = -1;
            ce = cre;
            lame = 1;
            line_length = norm(ce-cs);
            theta = atan2((ce(2)-cs(2)),(ce(1)-cs(1)));
            theta2 = acos(2*R/line_length);
            q1 = Rz(theta+theta2-pi/2)*e1;
            w1 =  cs + R*Rz(theta+theta2)*e1;
            w2 = ce + R*Rz(theta+theta2-pi)*e1;
         case 4
            cs = cls;
            lams = -1;
            ce = cle;
            lame = -1;
            q1 = (ce-cs)/norm(ce-cs);
            w1 = cs + R*Rz(pi/2)*q1;
            w2 = ce + R*Rz(pi/2)*q1;
    end
    w3 = pe;
    q3 = Rz(chie)*e1;
    
    % assign path variables
    dubinspath.ps   = ps;
    dubinspath.chis = chis;
    dubinspath.pe   = pe;
    dubinspath.chie = chie;
    dubinspath.R    = R;
    dubinspath.L    = L;
    dubinspath.cs   = cs;
    dubinspath.lams = lams;
    dubinspath.ce   = ce;
    dubinspath.lame = lame;
    dubinspath.w1   = w1;
    dubinspath.q1   = q1;
    dubinspath.w2   = w2;
    dubinspath.w3   = w3;
    dubinspath.q3   = q3;
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = Rz(theta)
% Rz - rotation matrix about the z axis of angle theta


    R = [cos(theta), -sin(theta), 0;
         sin(theta),  cos(theta), 0;
         0,           0,          1];
end
