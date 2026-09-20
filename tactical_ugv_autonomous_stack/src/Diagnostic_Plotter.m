%% Diagnostic Plotter 
% Intent:
%   This code is meant to:
%     - Read Diagnostic data logs from the darpa autonomous stack
%     - Plot Trajectory data vs Planned path to test open loop MPC
%     performance
% 
% Author: Barr Shlagman

clearvars;
close all;
clc;


%% Get Data from directory

% Choose Diagnostic directory
dataDirectory = 'Diagnostic_Logs';%uigetdir(pwd, 'Diagnostic_Logs');

if isequal(dataDirectory, 0)
    error('No diagnostic log directory was selected.');
end

logFiles = dir(fullfile(dataDirectory,'*DATA_Log.txt'));

if isempty(logFiles)
    error('No diagnostic log files were found in the selected directory.');
end

for i = 1:length(logFiles)
    % 1. Get the specific file name by indexing the struct array first
    currentFileName = logFiles(i).name;
    
    % 2. Combine the directory path with the file name
    fullPath = fullfile(dataDirectory, currentFileName);
    
    opts = detectImportOptions(fullPath);
    opts.Delimiter = {' ',',', ';'};

    % 3. Read the data into the cell array using curly braces
    C{i} = readcell(fullPath, opts);
end

disp('Order for data Array:')
disp('C{1} = Dgnstc_GOAL_GENERATION_DATA_Log.txt')
disp('C{2} = Dgnstc_PLANNER_DATA_Log.txt')
disp('C{3} = Dgnstc_TRAJECTORY_PLANNER_DATA_Log.txt')
disp('C{4} = Dgnstc_TRAJECTORY_PLANNER_GOAL_DATA_Log.txt')
disp('C{5} = Dgnstc_TRAJECTORY_PLANNER_POLICY_DATA_Log.txt')

%% Convert to mat formatting for plotter
% MPC Points to plot
M_points = 150;

GoalData = cell2mat(C{1}(:,8:9));
PathData = cell2mat(C{2}(:,2:3));
TrajectoryData(:,1) = cell2mat(C{3}(1:M_points,1));
TrajectoryData(:,2) = cell2mat(C{3}(1:M_points,2));

%% Star for goal point
% 1. Define the center coordinate and size of your star
x_center = 0;
y_center = 0;
star_size = 0.25; 

% 2. Calculate the geometry of a perfect 5-point star
r_outer = star_size;
r_inner = star_size * (3 - sqrt(5)) / 2; 

% FIX: Use linspace to guarantee exactly 10 points around the circle
angles = linspace(pi/2, pi/2 + 2*pi, 11);
angles(end) = []; % Remove the duplicate 11th point so it matches the 10 radii

% Alternate radiuses between inner and outer points (10 elements total)
radii = repmat([r_outer, r_inner], 1, 5);

% Convert polar coordinates to X and Y matrices (Both now have exactly 10 elements)
x_points = GoalData(1,1) + radii .* cos(angles);
y_points = GoalData(1,2) + radii .* sin(angles);


%% Plotting


figure('Name','Open Loop MPC Path Tracking');
plot(PathData(:,1), PathData(:,2), '-*','Color','r', 'LineWidth', 1.5);
hold on;
plot(TrajectoryData(1:M_points,1), TrajectoryData(1:M_points,2), '-b', 'LineWidth', 1.5);
% Explicitly label your X and Y coordinates so MATLAB accepts the styling pairs
patch('XData', x_points, 'YData', y_points, 'FaceColor', '#FFD700', 'EdgeColor', '#FF8C00', 'LineWidth', 1.5);
plot(2.13,3.7,'o','MarkerSize',80,'LineWidth',1.5)
xlabel('$x(t)$','Interpreter','latex','FontSize',14);
ylabel('$y(t)$','Interpreter','latex','FontSize',14);
title('Open Loop MPC Path Tracking')
legend('Planned Path','MPC Trajectory','Goal Point','Obstacle','Location','northwest');
grid on

%% Collision avoidance planes (first iteration only)
% Dgnstc_COLLISION_AVOIDANCE_Log.txt holds one block of rows "a, b, c, d" per constraint-generation
% cycle, one row per sector, each describing the plane a*x + b*y + c*z = d. Only the first cycle is drawn.
%
% Frame: constraint_generation builds its obstacle points from the voxel map as (0.2*k, 0.2*i, 0.2*j)
% (constraint.cpp), i.e. with x and y swapped relative to the odom frame used by the path and trajectory
% above, so the first two coefficients are swapped below before drawing.
%
% The planes are cut at height zSlice (the vehicle's start height from the goal generation log) to get a
% line in the x-y plane. Change zSlice to look at a different height.
% cgFile = fullfile(dataDirectory, 'Dgnstc_COLLISION_AVOIDANCE_Log.txt');
% zSlice = 0;
% if isnumeric(C{1}{1,7})
%     zSlice = C{1}{1,7};
% end
% 
% if isfile(cgFile)
%     planes = readmatrix(cgFile, 'Delimiter', ',');
%     planes = planes(all(isfinite(planes), 2), 1:4);
% 
%     if ~isempty(planes)
%         % First cycle = rows up to the next time the first row's plane normal shows up again
%         sameNormal = find(all(abs(planes(:,1:3) - planes(1,1:3)) < 1e-6, 2));
%         if numel(sameNormal) >= 2
%             firstCycle = planes(1:sameNormal(2)-1, :);
%         else
%             firstCycle = planes;
%         end
% 
%         ax = gca;
%         hold(ax, 'on');
%         xl = xlim(ax);
%         yl = ylim(ax);
%         xlim(ax, xl);   % freeze the current view so long lines do not rescale the plot
%         ylim(ax, yl);
% 
%         for p = 1:size(firstCycle, 1)
%             A = firstCycle(p,2);                            % odom x coefficient (swapped)
%             B = firstCycle(p,1);                            % odom y coefficient (swapped)
%             D = firstCycle(p,4) - firstCycle(p,3)*zSlice;   % A*x + B*y = D at z = zSlice
% 
%             if abs(B) >= abs(A)
%                 xs = xl;
%                 ys = (D - A*xs) / B;
%             else
%                 ys = yl;
%                 xs = (D - B*ys) / A;
%             end
% 
%             if p == 1
%                 plot(ax, xs, ys, '--', 'Color', [0.75 0 0.75], 'LineWidth', 1.5, ...
%                     'DisplayName', 'Collision Avoidance Planes (iter 1)');
%             else
%                 plot(ax, xs, ys, '--', 'Color', [0.75 0 0.75], 'LineWidth', 1.5, ...
%                     'HandleVisibility', 'off');
%             end
%         end
%     end
% else
%     warning('Collision avoidance log not found: %s', cgFile);
% end

%% Cross-trek Error
% Compute cumulative arc length of planned path
dx = diff(PathData(:,1));
dy = diff(PathData(:,2));
s = [0; cumsum(sqrt(dx.^2 + dy.^2))];

% Nearest-point lookup via knnsearch
idx = knnsearch(PathData, TrajectoryData);

% Calculate orthogonal distance to nearest reference points
crosstrack_error = sqrt(sum((TrajectoryData - PathData(idx, :)).^2, 2));

