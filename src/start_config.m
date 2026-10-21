function q_start = start_config(wall_e, transformations)
%This function finds the best starting joint configuration for our robot
%Test the possible configurations for the first point of the circle and
%select the smallest maximum angle change.


% Find possible joint configurations for the first point of the circle
S = ur5e_analytic_IK(wall_e, transformations(:,:,1));

if isempty(S)
    error('No joint configuration found for the first point');
end

%Defining variables
best = inf; 
q_start = [];

%Test different possible starting configurations
for k = 1:size(S,1)
    try
        % Calculate the full path using the current configuration
        Q = Inverse_Kinematics(wall_e, S(k,:), transformations);
    catch ME
        fprintf('Configuration %d failed: %s\n', k, ME.message);
        continue;
    end

    %Compare changes between consecutive points for the six joints and save
    %the largest difference
    joint_changes = abs(diff(Q,1,1));
    step = max(joint_changes(:));
    
    %Save the starting configuration with the smallest maximum joint angle change
    if step < best
        best = step; 
        q_start = S(k,:); 
    end
end

% Check if a valid starting configuration was found 
if isempty(q_start)
    error('No possible starting configuration found');
end
% Display the maximum joint angle change for the selected configuration
fprintf('Start config chosen: max joint step %.1f deg per frame\n', rad2deg(best));
end
