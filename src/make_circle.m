function [poses, pts] = make_circle(c, R, m, h, N)
% MAKE_CIRCLE_POSES  Circle on a sphere with the tool z-axis along the surface normal.
%   Sphere: |p - c| = R.  Circle plane: unit normal mhat, offset h from the center.
%   Circle center p0 = c + h*mhat, radius r = sqrt(R^2 - h^2)
%   Path:  p(t) = p0 + r*(cos t * u + sin t * v)
%   Frame: n = (p - c)/R (outward normal), z = -n, x = tangent, y = z cross x
%   Returns poses (4x4xN) and the N points pts (Nx3).
c = c(:)'; mhat = m(:)' / norm(m);
p0 = c + h*mhat;  r = sqrt(R^2 - h^2);
a = [1 0 0]; if abs(mhat(1)) > 0.9, a = [0 1 0]; end
u = cross(mhat, a); u = u / norm(u);   % in-plane basis vector 1
v = cross(mhat, u);                    % in-plane basis vector 2
th = linspace(0, 2*pi, N);
poses = zeros(4,4,N);  pts = zeros(N,3);
for i = 1:N
    t = th(i);
    p = p0 + r*(cos(t)*u + sin(t)*v);  % point on circle (and on sphere)
    n = (p - c) / R;                   % outward unit normal
    x = -sin(t)*u + cos(t)*v;          % unit tangent (perpendicular to n)
    z = -n;                            % tool points along the (inward) normal
    y = cross(z, x);
    poses(:,:,i) = [x(:) y(:) z(:) p(:); 0 0 0 1];
    pts(i,:) = p;
end
end