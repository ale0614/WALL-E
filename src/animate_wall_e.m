function animate_wall_e(wall_e, movements, c, R, pts, video_file)
%This function animates the robot following the circular path of the sphere

figure; hold on; axis equal; grid on; view(130, 25);

drawScene = nargin >= 5;
%Draw the sphere and the desired circular path
if drawScene
    [sx, sy, sz] = sphere(40);
    surf(c(1)+R*sx, c(2)+R*sy, c(3)+R*sz, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    plot3(pts(:,1), pts(:,2), pts(:,3), 'r--');
    nrm = (pts - c(:)') / R;                  
end

%Display robot in starting position
wall_e.plot(movements(1,:), 'workspace', [-0.9 0.9 -0.2 1.2 -0.3 1.0], 'noname');

% Animate the robot moving through each point
recording = nargin >= 6 && ~isempty(video_file);

%If recording create the video file
if recording
    try
        vw = VideoWriter(video_file, 'MPEG-4');
    catch
        vw = VideoWriter(strrep(video_file, '.mp4', ''), 'Motion JPEG AVI');
    end
    vw.FrameRate = 20; open(vw);
end

%Create an array to store the robot's path
N = size(movements,1); trail = zeros(N,3);

% Animate the robot moving through each point
for i = 1:N
    wall_e.animate(movements(i,:));
    % Calculate the end-Effector position using forward kinematics
    T = DH_FK(wall_e, movements(i,:)); 

     % Store and draw the path followed by the robot
    trail(i,:) = T(1:3,4)';
    plot3(trail(1:i,1), trail(1:i,2), trail(1:i,3), 'b', 'LineWidth', 2);
    
    if drawScene
        % Show the sphere normal and the tool's direction
        quiver3(pts(i,1),pts(i,2),pts(i,3), nrm(i,1),nrm(i,2),nrm(i,3), 0.06, 'g');
        quiver3(T(1,4),T(2,4),T(3,4), T(1,3),T(2,3),T(3,3), 0.06, 'm');
    end
    drawnow;

    % Save the current frame if recording
    if recording, writeVideo(vw, getframe(gcf)); end
end

% Close the video file 
if recording, close(vw); end
end
