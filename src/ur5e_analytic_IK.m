function sols = ur5e_analytic_IK(wall_e, T)
% UR5E_ANALYTIC_IK  Closed-form IK for the UR5e (up to 8 solutions, one per row).
%   Used ONLY to pick a good starting configuration. The path itself is solved
%   with the toolbox's ikine. Joint angles are returned wrapped to [-pi, pi].
%   Standard DH: A_i = Rz(q_i) Tz(d_i) Tx(a_i) Rx(alpha_i); a2, a3 < 0 for the UR5e.
L = wall_e.links;
d = zeros(1,6); a = d; al = d;
for j = 1:6, d(j) = L(j).d; a(j) = L(j).a; al(j) = L(j).alpha; end

T = double(wall_e.base) \ T;               % target pose in the robot's base frame
n = T(1:3,1); o = T(1:3,2); ap = T(1:3,3); p6 = T(1:3,4);
p5 = p6 - d(6)*ap;                         % wrist-2 center
rho = hypot(p5(1), p5(2));
sols = zeros(0,6);
if rho < abs(d(4)), return; end            % not reachable

phi = atan2(p5(2), p5(1));  k = asin(d(4)/rho);
for q1 = [phi + k, phi + pi - k]                       % 2 shoulder solutions
    z1 = [sin(q1); -cos(q1); 0];
    c5 = (p6.'*z1 - d(4)) / d(6);
    if abs(c5) > 1 + 1e-12, continue; end
    for q5 = [acos(max(-1,min(1,c5))), -acos(max(-1,min(1,c5)))]   % wrist up/down
        s5 = sin(q5);
        if abs(s5) < 1e-9, continue; end               % wrist singularity
        q6 = atan2(-(z1.'*o)/s5, (z1.'*n)/s5);
        % M = A2*A3*A4 = inv(A1) * T * inv(A6) * inv(A5)
        M = dh_A(q1,d(1),a(1),al(1)) \ T / dh_A(q6,d(6),a(6),al(6)) / dh_A(q5,d(5),a(5),al(5));
        th234 = atan2(M(2,1), M(1,1));  px = M(1,4);  py = M(2,4);
        c3 = (px^2 + py^2 - a(2)^2 - a(3)^2) / (2*a(2)*a(3));
        if abs(c3) > 1 + 1e-12, continue; end
        for q3 = [acos(max(-1,min(1,c3))), -acos(max(-1,min(1,c3)))]   % elbow up/down
            q2 = atan2(py, px) - atan2(a(3)*sin(q3), a(2) + a(3)*cos(q3));
            q4 = th234 - q2 - q3;
            sols(end+1,:) = [q1 q2 q3 q4 q5 q6]; %#ok<AGROW>
        end
    end
end
sols = mod(sols + pi, 2*pi) - pi;
end

function A = dh_A(th, d, a, al)
ct = cos(th); st = sin(th); ca = cos(al); sa = sin(al);
A = [ct, -st*ca,  st*sa, a*ct;
    st,  ct*ca, -ct*sa, a*st;
    0,   sa,     ca,    d;
    0,   0,      0,     1];
end
