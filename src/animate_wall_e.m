function animate_wall_e(wall_e, movements, c, R, pts, video_file)
% ANIMATE_WALL_E  Animate the robot drawing the circle on the sphere.
%   With c, R, pts it also draws the sphere, the desired circle, the traced path, the
%   sphere normals (green) and the tool z-axis (magenta); the two arrows should
%   point in opposite directions in every frame (tool z = -normal).
%   Optional video_file (e.g. 'wall_e.mp4') records the animation.
figure; hold on; axis equal; grid on; view(130, 25);
drawScene = nargin >= 5;
if drawScene
    [sx, sy, sz] = sphere(40);
    surf(c(1)+R*sx, c(2)+R*sy, c(3)+R*sz, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    plot3(pts(:,1), pts(:,2), pts(:,3), 'r--');
    nrm = (pts - c(:)') / R;                  % outward unit normals (N x 3)
end
wall_e.plot(movements(1,:), 'workspace', [-0.9 0.9 -0.2 1.2 -0.3 1.0], 'noname');

recording = nargin >= 6 && ~isempty(video_file);
if recording
    try
        vw = VideoWriter(video_file, 'MPEG-4');
    catch
        vw = VideoWriter(strrep(video_file, '.mp4', ''), 'Motion JPEG AVI');
    end
    vw.FrameRate = 20; open(vw);
end

N = size(movements,1); trail = zeros(N,3);
for i = 1:N
    wall_e.animate(movements(i,:));
    T = DH_FK(wall_e, movements(i,:));        % plain 4x4 (avoids SE3-object indexing)
    trail(i,:) = T(1:3,4)';
    plot3(trail(1:i,1), trail(1:i,2), trail(1:i,3), 'b', 'LineWidth', 2);
    if drawScene
        quiver3(pts(i,1),pts(i,2),pts(i,3), nrm(i,1),nrm(i,2),nrm(i,3), 0.06, 'g');
        quiver3(T(1,4),T(2,4),T(3,4), T(1,3),T(2,3),T(3,3), 0.06, 'm');
    end
    drawnow;
    if recording, writeVideo(vw, getframe(gcf)); end
end
if recording, close(vw); end
end
