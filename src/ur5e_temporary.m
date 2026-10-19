function wall_e= ur5e_temporary()

a = [0 -0.425 -0.3922 0 0 0];
d = [0.1625 0 0 0.1333 0.0997 0.0996];
alpha = [pi/2 0 0 pi/2 -pi/2 0];

for i = 1:6
    L(i) = Link([0 d(i) a(i) alpha(i)], 'standard');
end

wall_e = SerialLink(L, 'name', 'UR5e');
wall_e.base = eye(4);

for i = 1:6
    L(i) = Link([0 d(i) a(i) alpha(i)], 'standard');

    % Set joint limits in radians
    L(i).qlim = [-2*pi 2*pi];
end

end


