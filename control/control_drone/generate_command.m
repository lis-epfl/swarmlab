function [command] = generate_command(drone_type, autopilot_version, time)
% GENERATE_COMMAND - function that generates commands for a fixed_wing or a
% quadcopter drone. This function is used to test the autopilots of the
% drones, the commanded values here are hard-coded.

if drone_type == "fixed_wing" 
        % command set
        command = [0 100 0.26 0]';
elseif drone_type == "quadcopter" || drone_type == "point_mass"
        flag = mod(floor(time/3),2);

        switch autopilot_version
            case 1 % attitude control
                if flag == 0
                    command = [0 0 0 0.5]';
                else
                    command = [0 0 0 -0.5]';
                end
            case 2 % velocity controller
                if flag == 0
                    command = [0 0 6 0]';
                else
                    command = [0 0 -6 0]';
                end
            case 3 % acceleration controller
                command = [0.5 0 0 0]';
        end
        
end

end