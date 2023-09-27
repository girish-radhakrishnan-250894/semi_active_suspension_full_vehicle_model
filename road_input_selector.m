function [time_stamps,road_input_xr, road_input_zr] = road_input_selector(simulation_time , road_mode, u)
    %% UNTITLED2 
    % mode 1 - flat road
    % mode 2 - step input of 10 mm
    % mode 3 - random road input
    % mode 4 - speedbump
    time_accuracy = 0.01; % determines minimum time step

    if road_mode == 1
        % input.time =   [0 2 4 6 8 10 12 14 16 18 20];
        % input.z_r  = 3*[0 0 5 5 0  0  0  0  0  0  0]*1e-3;
        time_stamps = 0:time_accuracy:simulation_time;
        road_input_xr = time_stamps.*u;
        road_input_zr = zeros(1,numel(time_stamps));
    
    elseif road_mode == 2
        input = frest.createStep('StepTime',0.5,'StepSize',0.01,'FinalTime',simulation_time);
        time_stamps = transpose(input.Time);
        road_input_xr = time_stamps.*u;
        road_input_zr = transpose(input.Data);
    
    elseif road_mode == 3

        road_input_xr = 0:(time_accuracy*u):(simulation_time * u); % 3 km of road
        n = length(road_input_xr);
        road_input_zr = cumsum(0.001*(10*rand(1,n)-1));
        % start and end with zr equal to zero
        road_input_zr = road_input_zr - road_input_zr(1)- road_input_xr*(road_input_zr(n)-road_input_zr(1))/road_input_xr(n);
        time_stamps = road_input_xr ./ u;

    elseif road_mode == 4

        [time_stamps, road_input_xr, road_input_zr] = speedbump_creator(simulation_time, u, time_accuracy);
    end
end