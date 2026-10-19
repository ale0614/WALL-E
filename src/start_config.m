function q_start = start_config(wall_e, transformations)
% START_CONFIG  Choose the starting joints for pose 1.
%   Gets the (up to 8) exact solutions for pose 1, traces the whole circle from each
%   with Inverse_Kinematics (toolbox ikine), and keeps the smoothest valid one.
S = ur5e_analytic_IK(wall_e, transformations(:,:,1));
if isempty(S)
    error('Pose 1 is not reachable: move the base closer or shrink the circle.');
end
best = inf; q_start = [];
for k = 1:size(S,1)
    try
        Q = Inverse_Kinematics(wall_e, S(k,:), transformations);
    catch
        continue;                    % failed or hit a joint limit: try the next one
    end
    step = max(max(abs(diff(Q,1,1))));
    if step < best, best = step; q_start = S(k,:); end
end
if isempty(q_start)
    error('No start gives a full path within limits: move the base or shrink the circle.');
end
fprintf('Start config chosen: max joint step %.1f deg per frame\n', rad2deg(best));
end
