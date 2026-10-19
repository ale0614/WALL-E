def make_circle_poses(c, R, m, h, N):
    c = np.asarray(c, float); m = np.asarray(m, float); m = m/np.linalg.norm(m)
    p0 = c + h*m;  r = np.sqrt(R**2 - h**2)
    a = np.array([1,0,0]) if abs(m[0]) < 0.9 else np.array([0,1,0])
    u = np.cross(m, a); u /= np.linalg.norm(u);  v = np.cross(m, u)
    poses = []
    for t in np.linspace(0, 2*np.pi, N):
        p = p0 + r*(np.cos(t)*u + np.sin(t)*v)
        n = (p - c)/R
        x = -np.sin(t)*u + np.cos(t)*v
        z = -n;  y = np.cross(z, x)
        T = np.eye(4); T[:3,:3] = np.column_stack([x, y, z]); T[:3,3] = p
        poses.append(SE3(T))
    return poses