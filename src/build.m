function wall_e = build(base_pos)
% BUILD_UR5E  Universal Robots UR5e as a Peter Corke SerialLink (standard DH).
%   DH values (meters) are Universal Robots' published UR5e parameters:
%   https://www.universal-robots.com/articles/ur/parameters-for-calculations-of-kinematics-and-dynamics/
%   Joint limits are +/-360 deg on all six joints (UR5e datasheet).
%
%       Joint |   theta |    d      |    a     | alpha
%         1   |   q1    |  0.1625   |   0      |  pi/2
%         2   |   q2    |  0        | -0.4250  |  0
%         3   |   q3    |  0        | -0.3922  |  0
%         4   |   q4    |  0.1333   |   0      |  pi/2
%         5   |   q5    |  0.0997   |   0      | -pi/2
%         6   |   q6    |  0.0996   |   0      |  0
if nargin < 1, base_pos = [0 0 0]; end
d1 = 0.1625; a2 = -0.425; a3 = -0.3922; d4 = 0.1333; d5 = 0.0997; d6 = 0.0996;
qlim = [-2*pi 2*pi];

L(1) = Revolute('d', d1, 'a', 0,  'alpha',  pi/2, 'qlim', qlim);
L(2) = Revolute('d', 0,  'a', a2, 'alpha',  0,    'qlim', qlim);
L(3) = Revolute('d', 0,  'a', a3, 'alpha',  0,    'qlim', qlim);
L(4) = Revolute('d', d4, 'a', 0,  'alpha',  pi/2, 'qlim', qlim);
L(5) = Revolute('d', d5, 'a', 0,  'alpha', -pi/2, 'qlim', qlim);
L(6) = Revolute('d', d6, 'a', 0,  'alpha',  0,    'qlim', qlim);

wall_e = SerialLink(L, 'name', 'UR5e');
wall_e.base = transl(base_pos);       % base placed outside the sphere
end
