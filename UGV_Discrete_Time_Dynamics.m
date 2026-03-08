Pvx = [1 5 10 15 20 25 30 35];

fid = fopen('UGV_Dynamics_All.txt', 'w');

for i =  1:length(Pvx)
    % Vehicle parameters
    m = 5;        % vehicle mass (kg)
    Iz = 12;       % yaw moment of inertia (kg·m²)
    a = .15;        % CG to front axle (m)
    b = .15;        % CG to rear axle (m)
    Cf = 70;       % front cornering stiffness (N/rad)
    Cr = 70;       % rear cornering stiffness (N/rad)
    % Pvx = 10;      % constant speed (m/s)

    % Continuous lateral A and B
    Ac = [0,  1,  0,  0;
        0,  -2*(Cf+Cr)/(m*Pvx(i)),  0,  -Pvx(i) - 2*(a*Cf - b*Cr)/(m*Pvx(1,i));
        0,  0,  0,  1;
        0,  -2*(a*Cf - b*Cr)/(Iz*Pvx(i)),  0,  -2*(a^2*Cf + b^2*Cr)/(Iz*Pvx(i))];

    Bc = [0; 2*Cf/m; 0; 2*a*Cf/Iz];

    % Discretize
    Ts = 0.1;       % your MPC timestep (s)
    sys_c = ss(Ac, Bc, eye(4), 0);
    sys_d = c2d(sys_c, Ts, 'zoh');

    Ad = sys_d.A
    Bd = sys_d.B


    % writematrix(Ad,sprintf("Ad_%s.txt",num2str(Pvx(i))),"Delimiter", "\t");% "delimiter", "\t", "precision", "%.6f");  % 6 decimals, no exponent
    % writematrix(Bd, "Bd.txt","Delimiter", "\t");%  "delimiter", "\t", "precision", "%.6f");  % 6 decimals, no exponent
    %save("UGV_Discrete_Time_Dynamics.txt","Bd","Ad","-ascii", "precision", "%.6f")
    fprintf(fid, '////////////////////////////////////////////////\n');
    fprintf(fid, '// Ad (n x n): Pvx = %g m/s\n', Pvx(i));
    for r = 1:size(Ad,1)
        fprintf(fid, '%s\n', num2str(Ad(r,:), '%-12.6f'));
    end

    fprintf(fid, '\n');
end

fprintf(fid, '////////////////////////////////////////////////\n');
fprintf(fid, '// Bd (n x m): Pvx = %g m/s\n', Pvx(i));
for r = 1:size(Bd,1)
    fprintf(fid, '%s\n', num2str(Bd(r,:), '%-12.6f'));
end

fclose(fid);