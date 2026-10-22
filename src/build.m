function wall_e = build(base_pos)
% Create the UR5e robot model using its DH parameters

% Set the default base position and link dimensions
if nargin < 1, base_pos = [0 0 0]; end
d1 = 0.1625; a2 = -0.425; a3 = -0.3922; d4 = 0.1333; d5 = 0.0997; d6 = 0.0996;
qlim = [-2*pi 2*pi];

% Create the six revolute joints using the DH parameters
L(1) = Revolute('d', d1, 'a', 0,  'alpha',  pi/2, 'qlim', qlim);
L(2) = Revolute('d', 0,  'a', a2, 'alpha',  0,    'qlim', qlim);
L(3) = Revolute('d', 0,  'a', a3, 'alpha',  0,    'qlim', qlim);
L(4) = Revolute('d', d4, 'a', 0,  'alpha',  pi/2, 'qlim', qlim);
L(5) = Revolute('d', d5, 'a', 0,  'alpha', -pi/2, 'qlim', qlim);
L(6) = Revolute('d', d6, 'a', 0,  'alpha',  0,    'qlim', qlim);

% Combine the six joints into the UR5e robot model
wall_e = SerialLink(L, 'name', 'UR5e');

% Place the robot at the specified base position
wall_e.base = transl(base_pos);       
end
