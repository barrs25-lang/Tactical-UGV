% Vehicle parameters
m = 5;        % vehicle mass (kg)
Iz = 12;       % yaw moment of inertia (kg·m²)
a = .15;        % CG to front axle (m)
b = .15;        % CG to rear axle (m)
Cf = 70;       % front cornering stiffness (N/rad)
Cr = 70;       % rear cornering stiffness (N/rad)
Pvx = 10;      % constant speed (m/s)

% Continuous lateral A and B
Ac = [0,  1,  0,  0;
      0,  -2*(Cf+Cr)/(m*Pvx),  0,  -Pvx - 2*(a*Cf - b*Cr)/(m*Pvx);
      0,  0,  0,  1;
      0,  -2*(a*Cf - b*Cr)/(Iz*Pvx),  0,  -2*(a^2*Cf + b^2*Cr)/(Iz*Pvx)];

Bc = [0; 2*Cf/m; 0; 2*a*Cf/Iz];

% Discretize
Ts = 0.1;       % your MPC timestep (s)
sys_c = ss(Ac, Bc, eye(4), 0);
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A
Bd = sys_d.B