function animate_wall_e(wall_e, movements)

% Open a figure for the robot animation
figure; hold on; axis equal; grid on; view(130, 25);

if nargin >= 5
    [sx, sy, sz] = sphere(40);
    surf(c(1)+R*sx, c(2)+R*sy, c(3)+R*sz, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    plot3(pts(:,1), pts(:,2), pts(:,3), 'r--');
    nrm = (pts - c(:)) / R;
end 

% Animate the robot using the calculated joint angles
wall_e.plot(movements(1,:), 'workspace', [-0.9 0.9 -0.2 1.2 -0.3 1.0], 'noname');
N = size(movements,1); trail = zeros(N,3);
    for i = 1:N
        wall_e.animate(movements(i,:));
        T = wall_e.fkine(movements(i,:)); trail(i,:) = T(1:3,4)';
        plot3(trail(1:i,1), trail(1:i,2), trail(1:i,3), 'b', 'LineWidth', 2);
        if nargin >= 5
            quiver3(pts(i,1),pts(i,2),pts(i,3), nrm(i,1),nrm(i,2),nrm(i,3), 0.06, 'g');
            quiver3(T(1,4),T(2,4),T(3,4), T(1,3),T(2,3),T(3,3), 0.06, 'm');
        end
        drawnow;
    end

end