function plot_state(self, time, output_rate)
% plot_state - Plot the state variables of the drone.
%
% Inputs:
%   time
%   output_rate
%

% TODO: use function deg2rad instead of the scaling factor.
% TODO: 'persistent' vars should be avoided. Check if possible. Should be
% done by creating a DroneStateViewer

    if mod(time, output_rate) == 0 && self.drone_type ~= "point_mass"
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        persistent fig_handle

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Rename variables

        % State variables
        pn          = self.pos_ned(1);          % North position                (meters)
        pe          = self.pos_ned(2);          % East position                 (meters)
        h           = -self.pos_ned(3);         % altitude                      (meters)
        vx          = self.vel_xyz(1);          % body velocity along x-axis    (meters/s)
        vy          = self.vel_xyz(2);          % body velocity along y-axis    (meters/s)
        vz          = self.vel_xyz(3);          % body velocity along z-axis    (meters/s)
        phi         = 180/pi*self.attitude(1);  % roll angle                    (degrees)   
        theta       = 180/pi*self.attitude(2);  % pitch angle                   (degrees)
        psi         = 180/pi*self.attitude(3);  % yaw angle                     (degrees)
        p           = 180/pi*self.rates(1);     % body angular rate along x-axis (degrees/s)
        q           = 180/pi*self.rates(2);     % body angular rate along y-axis (degrees/s)
        r           = 180/pi*self.rates(3);     % body angular rate along z-axis (degrees/s)

        % Air data
        Va          = self.airdata(1);          % airspeed                  (m/s)
        alpha       = 180/pi*self.airdata(2);   % angle of attack           (degrees)
        beta        = 180/pi*self.airdata(3);   % side slip angle           (degrees)
        wn          = self.airdata(4);          % wind in the North direction
        we          = self.airdata(5);          % wind in the East direction
        wd          = self.airdata(6);          % wind in the Down direction

        % Commanded variables
        pn_c        = self.full_command(1);     % commanded North position  (meters)
        pe_c        = self.full_command(2);     % commanded East position   (meters)
        h_c         = self.full_command(3);     % commanded altitude        (meters)
        vn_c        = self.full_command(4);     % commanded vn              (m/s)
        ve_c        = self.full_command(5);     % commanded ve              (m/s)
        vd_c        = self.full_command(6);     % commanded vd              (m/s)
        an_c        = self.full_command(7);
        ae_c        = self.full_command(8);
        ad_c        = self.full_command(9);
        Va_c        = self.full_command(10);        % commanded airspeed        (meters/s)
        alpha_c     = 180/pi*self.full_command(11); % commanded angle of attack (degrees)
        beta_c      = 180/pi*self.full_command(12); % commanded side slip angle (degrees)
        phi_c       = 180/pi*self.full_command(13); % commanded roll angle      (degrees)   
        theta_c     = 180/pi*self.full_command(14); % commanded pitch angle     (degrees)
        psi_c       = 180/pi*self.full_command(15); % commanded yaw             (degrees)
        chi_c       = 180/pi*self.full_command(16); % commanded course          (degrees)
        p_c         = 180/pi*self.full_command(17); % commanded body angular rate along x-axis (degrees/s)
        q_c         = 180/pi*self.full_command(18); % commanded body angular rate along y-axis (degrees/s)
        r_c         = 180/pi*self.full_command(19); % commanded body angular rate along z-axis (degrees/s)

        % Estimated variables
        pn_hat      = self.z_hat(1);        % estimated North position  (meters)
        pe_hat      = self.z_hat(2);        % estimated East position   (meters)
        h_hat       = self.z_hat(3);        % estimated altitude        (meters)
        % u_hat     =  self.z_hat(4);
        % v_hat     =  self.z_hat(5);
        % w_hat     =  self.z_hat(6);
        Va_hat      = self.z_hat(7);        % estimated airspeed        (meters/s)
        alpha_hat   = 180/pi*self.z_hat(8); % estimated angle of attack (degrees)
        beta_hat    = 180/pi*self.z_hat(9); % estimated side slip angle (degrees)
        phi_hat     = 180/pi*self.z_hat(10);% estimated roll angle      (degrees)
        theta_hat   = 180/pi*self.z_hat(11);% estimated pitch angle     (degrees)
        psi_hat     = 180/pi*self.z_hat(12);
        chi_hat     = 180/pi*self.z_hat(13);% estimated course          (degrees)
        p_hat       = 180/pi*self.z_hat(14);% estimated body angular rate along x-axis (degrees/s)
        q_hat       = 180/pi*self.z_hat(15);% estimated body angular rate along y-axis (degrees/s)
        r_hat       = 180/pi*self.z_hat(16);% estimated body angular rate along z-axis (degrees/s)
        % Vg_hat      = self.z_hat(17);     % estimated groundspeed
        % wn_hat      = self.z_hat(18);     % estimated North wind
        % we_hat      = self.z_hat(19);     % estimated East wind
        % bx_hat      = self.z_hat(20);     % estimated x-gyro bias
        % by_hat      = self.z_hat(21);     % estimated y-gyro bias
        % bz_hat      = self.z_hat(22);     % estimated z-gyro bias

        % Delta
        if self.drone_type == "fixed_wing"
            de          = 180/pi*self.delta(1);     % elevator angle   (degrees)
            da          = 180/pi*self.delta(2);     % aileron angle    (degrees)
            dr          = 180/pi*self.delta(3);     % rudder angle     (degrees)
            dt          = self.delta(4);            % throttle setting (unitless)
        elseif self.drone_type == "quadcopter"
            d1          = self.delta(1);     % throttle first propeller (unitless, between 0 and 1)
            d2          = self.delta(2);     % throttle second propeller
            d3          = self.delta(3);     % throttle third propeller
            d4          = self.delta(4);     % throttle fourth propeller
        end

        % Forces in the body frame
        F_xyz = self.forces;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Define persistent variables

        % Compute course angle
        chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);

        persistent pn_handle
        persistent pe_handle
        persistent h_handle
        persistent vn_handle
        persistent ve_handle
        persistent vd_handle
        persistent Va_handle
        persistent alpha_handle
        persistent beta_handle
        persistent phi_handle
        persistent theta_handle
        if self.drone_type == "fixed_wing"
            persistent chi_handle
        elseif self.drone_type == "quadcopter"
            persistent psi_handle
        end
        persistent p_handle
        persistent q_handle
        persistent r_handle
        if self.drone_type == "fixed_wing"
            persistent de_handle
            persistent da_handle
            persistent dr_handle
            persistent dt_handle
        elseif self.drone_type == "quadcopter"
            persistent d1_handle
            persistent d2_handle
            persistent d3_handle
            persistent d4_handle
        end

        persistent an_handle
        persistent ae_handle
        persistent ad_handle

        Rbi = Rb2i(phi*pi/180, theta*pi/180, psi*pi/180);
        v_ned = Rbi*[vx, vy, vz]';
        vn = v_ned(1);
        ve = v_ned(2);
        vd = v_ned(3);

        a_ned =  Rbi*F_xyz/self.p_drone.mass;
        an = a_ned(1);
        ae = a_ned(2);
        ad = a_ned(3);


        % first time function is called, initialize plot and persistent vars
        if time == 0
            x0 = 960; 
            y0 = 0; 
            width = 960;
            height = 1080;
            fig_handle = figure('Name','Drone state viewer', ...
                'NumberTitle','off', 'position', [x0, y0, width, height]);
            tg = uitabgroup; % tabgroup


            %%%% POSITION %%%%
            position_tab = uitab(tg, 'title', 'Position');
            axes('Parent',position_tab);
            subplot(3,1,1)
            hold on
            %       pn_handle = graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', []);
            if isinf(pn_c)
                pn_handle = graph_y_yhat(time, pn, pn, 'p_n [m]', []);
            else
                pn_handle = graph_y_yhat_yd(time, pn, pn_hat, pn, 'p_n [m]', []);
            end

            subplot(3,1,2)
            hold on
            %       pe_handle = graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', []);
            pe_handle = graph_y_yhat_yd(time, pe, pe_hat, pe, 'p_e [m]', []);

            subplot(3,1,3)
            hold on
            h_handle = graph_y_yhat_yd(time, h, h_hat, h_c, 'h [m]', []);
            legend('real', 'estimated', 'command');


            %%%% VELOCITY %%%%
            velocity_tab = uitab(tg, 'title', 'Velocity');
            axes('Parent',velocity_tab);
            subplot(3,1,1)
            hold on
            vn_handle = graph_y_yhat_yd(time, vn, 0, vn_c, 'v_n [m/s]', []);

            subplot(3,1,2)
            hold on
            ve_handle = graph_y_yhat_yd(time, ve, 0, ve_c, 'v_e [m/s]', []);

            subplot(3,1,3)
            hold on
            vd_handle = graph_y_yhat_yd(time, - vd, 0, - vd_c, 'v_h [m/s]', []);
            legend('real', 'estimated', 'command');


            %%%% ACCELERATION %%%%
            accel_tab = uitab(tg, 'title', 'Acceleration');
            axes('Parent',accel_tab);
            subplot(3,1,1)
            hold on
            an_handle = graph_y_yd(time, an, an_c, 'a_n [m/s]', []);

            subplot(3,1,2)
            hold on
            ae_handle = graph_y_yd(time, ae, ae_c, 'a_e [m/s]', []);

            subplot(3,1,3)
            hold on
            ad_handle = graph_y_yd(time, ad, ad_c, 'a_d [m/s]', []);
            legend('real', 'command');


            %%%% AIRDATA %%%%
            airdata_tab = uitab(tg, 'title', 'Air data');
            axes('Parent',airdata_tab);
            subplot(3,1,1)
            hold on
            Va_handle = graph_y_yhat_yd(time, Va, Va_hat, Va_c, 'V_a [m/s]', []);

            subplot(3,1,2)
            hold on
            alpha_handle = graph_y_yhat_yd(time, alpha, alpha_hat, alpha_c, '\alpha [deg]', []);

            subplot(3,1,3)
            hold on
            beta_handle = graph_y_yhat_yd(time, beta, beta_hat, beta_c, '\beta [deg]', []);
            legend('real', 'estimated', 'command');


            %%%% ATTITUDE %%%%
            attitude_tab = uitab(tg, 'title', 'Attitude');
            axes('Parent',attitude_tab);
            subplot(3,1,1)
            hold on
            phi_handle = graph_y_yhat_yd(time, phi, phi_hat, phi_c, '\phi [deg]', []);

            subplot(3,1,2)
            hold on
            theta_handle = graph_y_yhat_yd(time, theta, theta_hat, theta_c, '\theta [deg]', []);

            subplot(3,1,3)
            hold on
            if self.drone_type == "fixed_wing"
                chi_handle = graph_y_yhat_yd(time, chi, chi_hat, chi_c, '\chi [deg]', []);
            elseif self.drone_type == "quadcopter"
                psi_handle = graph_y_yhat_yd(time, psi, psi_hat, psi_c, '\psi [deg]', []);
            end
            legend('real', 'estimated', 'command');


            %%%% ANGLES RATES %%%%
            anglerates_tab = uitab(tg, 'title', 'Angle rates');
            axes('Parent',anglerates_tab)
            subplot(3,1,1)
            hold on
            p_handle = graph_y_yhat_yd(time, p, p_hat, p_c, 'p [deg/s]', []);

            subplot(3,1,2)
            hold on
            q_handle = graph_y_yhat_yd(time, q, q_hat, q_c, 'q [deg/s]', []);

            subplot(3,1,3)
            hold on
            r_handle = graph_y_yhat_yd(time, r, r_hat, r_c, 'r [deg/s]', []);
            legend('real', 'estimated', 'command');


            %%%% ACTUATORS %%%%
            actuators_tab = uitab(tg, 'title', 'Actuators');
            axes('Parent',actuators_tab)
            subplot(4,1,1)
            hold on
            if self.drone_type == "fixed_wing"
                de_handle = graph_y(time, de, [], 'b');
                ylabel('\delta_e [deg]')
            elseif self.drone_type == "quadcopter"
                d1_handle = graph_y(time, d2, [], 'b');
                ylim([0 1]);
                ylabel('\delta_T1 [0..1]')
            end

            subplot(4,1,2)
            hold on
            if self.drone_type == "fixed_wing"
                da_handle = graph_y(time, da, [], 'b');
                ylabel('\delta_a [deg]')
            elseif self.drone_type == "quadcopter"
                d2_handle = graph_y(time, d2, [], 'b');
                ylim([0 1]);
                ylabel('\delta_T2 [0..1]')
            end

            subplot(4,1,3)
            hold on
            if self.drone_type == "fixed_wing"
                dr_handle = graph_y(time, dr, [], 'b');
                ylabel('\delta_r [deg]')
            elseif self.drone_type == "quadcopter"
                d3_handle = graph_y(time, d3, [], 'b');
                ylim([0 1]);
                ylabel('\delta_T3 [0..1]')
            end

            subplot(4,1,4)
            hold on
            if self.drone_type == "fixed_wing"
                dt_handle = graph_y(time, dt, [], 'b');
                ylim([0 1]);
                ylabel('\delta_t [0..1]')
            elseif self.drone_type == "quadcopter"
                d4_handle = graph_y(time, d4, [], 'b');
                ylim([0 1]);
                ylabel('\delta_T4 [0..1]')
            end

        % At every other time step, redraw state variables
        else 
           graph_y_yhat_yd(time, pn, pn_hat, pn_c, 'p_n', pn_handle);
           graph_y_yhat_yd(time, pe, pe_hat, pe_c, 'p_e', pe_handle);
           graph_y_yhat_yd(time, h, h_hat, h_c, 'h', h_handle);
           graph_y_yhat_yd(time, vn, 0, vn_c, 'v_n', vn_handle);
           graph_y_yhat_yd(time, ve, 0, ve_c, 'v_e', ve_handle);
           graph_y_yhat_yd(time, - vd, - 0, - vd_c, 'v_h', vd_handle);
           graph_y_yd(time, an, an_c, 'a_n', an_handle);
           graph_y_yd(time, ae, ae_c, 'a_e', ae_handle);
           graph_y_yd(time, ad, ad_c, 'a_h', ad_handle);
           graph_y_yhat_yd(time, Va, Va_hat, Va_c, 'V_a', Va_handle);
           graph_y_yhat_yd(time, alpha, alpha_hat, alpha_c, '\alpha', alpha_handle);
           graph_y_yhat_yd(time, beta, beta_hat, beta_c, '\beta', beta_handle);
           graph_y_yhat_yd(time, phi, phi_hat, phi_c, '\phi', phi_handle);
           graph_y_yhat_yd(time, theta, theta_hat, theta_c, '\theta', theta_handle);
           if self.drone_type == "fixed_wing"
                graph_y_yhat_yd(time, chi, chi_hat, chi_c, '\chi', chi_handle);
           elseif self.drone_type == "quadcopter"
               graph_y_yhat_yd(time, psi, psi_hat, psi_c, '\psi', psi_handle);
           end
           graph_y_yhat_yd(time, p, p_hat, p_c, 'p', p_handle);
           graph_y_yhat_yd(time, q, q_hat, q_c, 'q', q_handle);
           graph_y_yhat_yd(time, r, r_hat, r_c, 'r', r_handle);
           if self.drone_type == "fixed_wing"
                graph_y(time, de, de_handle);
                graph_y(time, da, da_handle);
                graph_y(time, dr, dr_handle);
                graph_y(time, dt, dt_handle);
           elseif self.drone_type == "quadcopter"
                graph_y(time, d1, d1_handle);
                graph_y(time, d2, d2_handle);
                graph_y(time, d3, d3_handle);
                graph_y(time, d4, d4_handle);           
           end
        end


    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph y with lable mylabel
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle)
    handle    = plot(t,y,color);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph y and yd with lable mylabel
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle)
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'r--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
    grid on
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph y and yd with lable mylabel
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = graph_y_yhat(t, y, y_hat, lab, handle)
  
  if isempty(handle)
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,y_hat,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
    grid on
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),y_hat]);
    %drawnow
  end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle)
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',90.0);
    grid on
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
% saturates the input between high and low
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out=sat(in, low, high)

  if in < low
      out = low;
  elseif in > high
      out = high;
  else
      out = in;
  end
end