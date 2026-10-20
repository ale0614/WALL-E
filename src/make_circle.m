function [poses, pts] = make_circle(c, R, m, h, N)
% Create a circular path on the surface of a sphere while 
% calculating the position and orientation at each point

c = c(:)'; 
mhat = m(:)' / norm(m);

%Calculate the center and radius of the circle
p0 = c + h*mhat;  
r = sqrt(R^2 - h^2);
a = [1 0 0]; if abs(mhat(1)) > 0.9, a = [0 1 0]; end

% Calculate two directions to create the circular path
u = cross(mhat, a); u = u / norm(u);   
v = cross(mhat, u);                  
th = linspace(0, 2*pi, N);

%Create arrays
poses = zeros(4,4,N);  pts = zeros(N,3);

%Calculate the positions and orientations for each point on the circle
for i = 1:N
    %Get current angle
    t = th(i);
    %Calculate the positions of the current point
    p = p0 + r*(cos(t)*u + sin(t)*v);  
    %Find the direction from the center to the current point 
    n = (p - c) / R;
    %Calculate the tangent direction of the circle
    x = -sin(t)*u + cos(t)*v;        
    z = -n; 
    %Calculate our y-axis
    y = cross(z, x);

    %Store the position, orientations and coordinates of the current point
    poses(:,:,i) = [x(:) y(:) z(:) p(:); 0 0 0 1];
    pts(i,:) = p;
end

end