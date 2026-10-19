function q_start = find_start_config(wall_e, transformations, tries)
% FIND_START_CONFIG  Pick the starting joints (for pose 1) that give the smoothest
%   full-circle path within the joint limits. Pass the result to Inverse_Kinematics.
    if nargin < 3, tries = 200; end
    rng(1); best = inf; q_start = [];
    for k = 1:tries
        q0 = (rand(1,6) - 0.5) * 2*pi;
        q1 = wall_e.ikine(transformations(:,:,1), 'q0', q0, 'mask', [1 1 1 1 1 1]);
        if isempty(q1), continue; end
        try
            Q = Inverse_Kinematics(wall_e, q1(1,:), transformations);
        catch
            continue;                       % failed or hit a limit: try another seed
        end
        step = max(max(abs(diff(Q,1,1))));
        if step < best, best = step; q_start = q1(1,:); end
    end
    if isempty(q_start)
        error('No valid start found: move the base closer or shrink the circle.');
    end
    fprintf('Best start: max joint step %.1f deg per frame\n', rad2deg(best));
end