function animate_wall_e(wall_e, movements)

% Open a figure for the robot animation
figure;

% Animate the robot using the calculated joint angles
wall_e.plot(movements);

end