%% UR5e Robot Simulation with Automatic Obstacle Avoidance
% Rewritten for 2x2 multi-view and automatic path planning

clear; clc; close all;

%% ========================================================================
%% USER CONFIGURABLE PARAMETERS
%% ========================================================================

% ROBOT INITIAL CONFIGURATION
verticalConfig = [0; 0; 0; 0; 0; 0];  % [Joint1; Joint2; Joint3; Joint4; Joint5; Joint6] in radians

% TARGET POSITIONS [X, Y, Z, RotX, RotY, RotZ]
pickupPosition = [0.5, 0.3, 0.2, pi, 0, 0];     % Pickup target location
placePosition = [0.6, -0.4, 0.2, pi, 0, 0];    % Place target location

% OBSTACLE PARAMETERS
obstacles = [
    % [X_center, Y_center, Z_center, X_size, Y_size, Z_size]
    [0.5, -0.05, 0.25, 0.12, 0.3, 0.5];   % Main central obstacle
    [0.6, -0.25, 0.12, 0.08, 0.08, 0.24]; % Small side obstacle
];
obstacleColors = [
    [0.8, 0.2, 0.2];  % Red for main obstacle
    [0.6, 0.1, 0.1];  % Dark red for side obstacle
];
safetyMargin = 0.06;  % Safety margin around obstacles (meters)

% AUTOMATIC PATH PLANNING PARAMETERS
clearanceHeight = 0.5;     % Height to clear obstacles automatically
pathResolution = 0.05;     % Path planning resolution (meters)
maxPlanningAttempts = 50;  % Maximum attempts for automatic planning

% OBJECT PROPERTIES
objectSize = [0.05, 0.05, 0.05];                % Object dimensions [length, width, height]
objectColor = [1.0, 0.5, 0.0];                  % Object color [R, G, B] (orange)

% PATH PLANNING PARAMETERS
numWaypoints = 20;       % Number of waypoints per path segment
numSteps = 6;            % Interpolation steps between waypoints
pickupThreshold = 0.10;  % Distance threshold for pickup (meters)
placeThreshold = 0.12;   % Distance threshold for placement (meters)
maxIKAttempts = 20;      % Maximum IK solving attempts per waypoint

% VISUALIZATION PARAMETERS
updateEvery = 4;         % Update visualization every N frames
saveEvery = 2;           % Save image every N frames
imageQuality = 1.0;      % Image size factor (0.6 = 60% of original size)
pauseTime = 0.02;        % Pause between frames (seconds)

% WORKSPACE LIMITS
workspaceX = [-1.0, 1.0];    % X-axis limits for visualization
workspaceY = [-1.0, 1.0];    % Y-axis limits for visualization  
workspaceZ = [-1.0, 1.0];       % Z-axis limits for visualization

%% ========================================================================
%% SIMULATION START
%% ========================================================================

fprintf('=== UR5e Automatic Obstacle Avoidance Simulation ===\n');
fprintf('Pickup Position: [%.2f, %.2f, %.2f]\n', pickupPosition(1:3));
fprintf('Place Position:  [%.2f, %.2f, %.2f]\n', placePosition(1:3));
fprintf('Number of Obstacles: %d\n', size(obstacles, 1));
fprintf('Safety Margin: %.3f m\n', safetyMargin);
fprintf('Clearance Height: %.3f m\n', clearanceHeight);
fprintf('Automatic Path Planning: ENABLED\n');

%% Robot Setup
fprintf('\nSetting up UR5e robot...\n');

robot = loadrobot('universalUR5e');
robot.DataFormat = 'column';
currentConfig = verticalConfig;

% Calculate vertical gripper position
T_vertical = getTransform(robot, verticalConfig, 'tool0', 'base_link');
verticalGripperPos = T_vertical(1:3, 4);

%% Automatic Path Planning
fprintf('Planning automatic obstacle avoidance path...\n');

% Generate automatic waypoints using geometric path planning
automaticWaypoints = generateAutomaticPath(pickupPosition(1:3), placePosition(1:3), ...
                                         obstacles, safetyMargin, clearanceHeight, pathResolution);

fprintf('Generated %d automatic waypoints\n', size(automaticWaypoints, 1));

% Create complete path sequence
pathSequence = [
    verticalGripperPos';           % Start position
    pickupPosition(1:3);          % Pickup position
    automaticWaypoints;           % Automatically generated waypoints
    placePosition(1:3);           % Final place position
];

pathNames = cell(size(pathSequence, 1) - 1, 1);
pathNames{1} = 'Move to Pickup';
pathNames{2} = 'Pick Object';
for i = 3:length(pathNames)-1
    pathNames{i} = sprintf('Auto Navigate %d', i-2);
end
pathNames{end} = 'Place Object';

%% Compute IK for all waypoints with robust solving
ikSolver = inverseKinematics('RigidBodyTree', robot);
ikSolver.SolverParameters.AllowRandomRestart = true;
ikSolver.SolverParameters.MaxIterations = 1500;

fprintf('Computing IK solutions for automatic path...\n');
numWaypoints_total = size(pathSequence, 1);
waypointConfigs = cell(numWaypoints_total, 1);
waypointConfigs{1} = verticalConfig;

for i = 2:numWaypoints_total
    targetPos = pathSequence(i, :);
    targetPose = [targetPos, pi, 0, 0]; % Keep gripper pointing down
    
    fprintf('Computing IK for waypoint %d: [%.3f, %.3f, %.3f]\n', i, targetPos);
    
    % Use previous configuration as initial guess
    initialGuess = waypointConfigs{i-1};
    
    % Compute robust IK
    [config, success] = computeRobustIK(ikSolver, robot, targetPose, initialGuess, maxIKAttempts);
    
    if success
        waypointConfigs{i} = config;
        T_result = getTransform(robot, config, 'tool0', 'base_link');
        actualPos = T_result(1:3, 4);
        error = norm(actualPos - targetPos');
        fprintf('  ✓ IK solved successfully, error: %.4f m\n', error);
    else
        error('Failed to find IK solution for waypoint %d. Consider adjusting target positions.', i);
    end
end

% Verify all configurations
fprintf('Verifying waypoint configurations...\n');
for i = 1:numWaypoints_total
    waypointConfigs{i} = waypointConfigs{i}(:);  % Ensure column vector
    if length(waypointConfigs{i}) ~= 6
        error('Waypoint configuration %d has wrong size: %d (expected 6)', i, length(waypointConfigs{i}));
    end
end
fprintf('All waypoint configurations verified successfully!\n');

%% Pre-compute ALL Paths
fprintf('Pre-computing all joint paths...\n');
tic;

totalFrames = 0;
allJointPaths = cell(length(pathNames), 1);
allStepCounts = zeros(length(pathNames), 1);

% Generate all paths
for taskIdx = 1:length(pathNames)
    startConfig = waypointConfigs{taskIdx}(:);
    goalConfig = waypointConfigs{taskIdx + 1}(:);
    
    fprintf('Task %d (%s):\n', taskIdx, pathNames{taskIdx});
    
    configDiff = norm(goalConfig - startConfig);
    
    if configDiff < 0.01
        jointPath = repmat(startConfig, 1, 8);  % Pause for pick/place
        fprintf('  Small movement - creating pause\n');
    else
        jointPath = generateSmoothJointPath(startConfig, goalConfig, numWaypoints);
        fprintf('  Generated path with %d waypoints\n', size(jointPath, 2));
    end
    
    allJointPaths{taskIdx} = jointPath;
    allStepCounts(taskIdx) = size(jointPath, 2) * numSteps;
    totalFrames = totalFrames + allStepCounts(taskIdx);
end

fprintf('Total frames to compute: %d\n', totalFrames);

% Pre-allocate ALL arrays
allConfigs = zeros(6, totalFrames);
allEEPositions = zeros(3, totalFrames);
frameToTask = zeros(1, totalFrames);

% Pre-compute ALL configurations and EE positions
fprintf('Pre-computing %d total frames...\n', totalFrames);
globalFrameIdx = 1;

for taskIdx = 1:length(pathNames)
    jointPath = allJointPaths{taskIdx};
    
    for waypointIdx = 1:size(jointPath, 2)
        if globalFrameIdx == 1
            startConfigLocal = verticalConfig;
        else
            startConfigLocal = allConfigs(:, globalFrameIdx-1);
        end
        targetConfig = jointPath(:, waypointIdx);
        
        % Generate interpolated steps
        stepPath = generateSmoothJointPath(startConfigLocal, targetConfig, numSteps);
        
        % Store all steps
        for step = 1:numSteps
            if globalFrameIdx <= totalFrames
                allConfigs(:, globalFrameIdx) = stepPath(:, step);
                frameToTask(globalFrameIdx) = taskIdx;
                globalFrameIdx = globalFrameIdx + 1;
            end
        end
    end
end

% Compute all EE positions
fprintf('Computing all EE positions...\n');
for i = 1:totalFrames
    T_ee = getTransform(robot, allConfigs(:, i), 'tool0', 'base_link');
    allEEPositions(:, i) = T_ee(1:3, 4);
end

precomputeTime = toc;
fprintf('Pre-computation completed in %.2f seconds\n', precomputeTime);

%% Setup Visualization  
imgDir = 'ur5e_auto_obstacle_frames';
if exist(imgDir, 'dir'), rmdir(imgDir, 's'); end
mkdir(imgDir);

% Create figure with explicit size and properties for 2x2 layout
fig = figure('Name', 'UR5e Automatic Obstacle Avoidance - 2x2 Multi-View', ...
             'Position', [50, 50, 1600, 1200], ...
             'Renderer', 'opengl', ...
             'Color', 'white', ...
             'WindowState', 'maximized', ...
             'NumberTitle', 'off');

% Test the subplot layout immediately
fprintf('Testing 2x2 subplot layout...\n');
for i = 1:4
    subplot(2, 2, i);
    text(0.5, 0.5, sprintf('View %d', i), 'HorizontalAlignment', 'center', ...
         'FontSize', 16, 'FontWeight', 'bold');
    title(sprintf('Subplot %d', i));
end
pause(1); % Show test layout briefly
fprintf('2x2 layout test completed. All 4 subplots should be visible.\n');

%% Execution Loop
fprintf('Starting automatic obstacle avoidance simulation...\n');
tic;

objectAttached = false;
objectPosition = pickupPosition(1:3);
savedFrameCount = 0;

for frameIdx = 1:totalFrames
    currentConfig = allConfigs(:, frameIdx);
    eePos = allEEPositions(:, frameIdx);
    currentTask = frameToTask(frameIdx);
    
    % Object attachment logic
    distToPickup = norm(eePos - pickupPosition(1:3)');
    distToPlace = norm(eePos - placePosition(1:3)');
    
    % Pick up object when close to pickup position
    if ~objectAttached && distToPickup < pickupThreshold && currentTask >= 2
        objectAttached = true;
        fprintf('Frame %d: Object picked up (distance: %.4f m)\n', frameIdx, distToPickup);
    end
    
    % Place object when close to place position AND we're in the final task
    if objectAttached && currentTask == length(pathNames) && distToPlace < placeThreshold
        objectAttached = false;
        objectPosition = eePos;
        fprintf('Frame %d: Object placed (distance: %.4f m)\n', frameIdx, distToPlace);
    end
    
    % Update object position if attached
    if objectAttached
        currentObjectPos = eePos + [0; 0; 0.08];
    else
        currentObjectPos = objectPosition;
    end
    
    % 2x2 Multi-view visualization update
    if mod(frameIdx, updateEvery) == 0 || frameIdx == 1 || frameIdx == totalFrames
        % Make sure we're working with the right figure
        figure(fig);
        
        update2x2MultiView(robot, currentConfig, eePos, allEEPositions(:, 1:frameIdx), ...
                          objectAttached, currentObjectPos, objectSize, objectColor, ...
                          pickupPosition, placePosition, automaticWaypoints, ...
                          obstacles, obstacleColors, pathNames{currentTask}, ...
                          frameIdx, currentTask, workspaceX, workspaceY, workspaceZ);
        
        % Save frame
        if mod(frameIdx, saveEvery) == 0
            savedFrameCount = savedFrameCount + 1;
            filename = fullfile(imgDir, sprintf('frame_%04d.png', savedFrameCount));
            frame = getframe(fig);  % Use specific figure handle
            img = imresize(frame.cdata, imageQuality);
            imwrite(img, filename, 'png', 'Compression', 'high');
        end
        
        drawnow;  % Force immediate drawing
        pause(pauseTime);
    end
end

executionTime = toc;
totalTime = precomputeTime + executionTime;

%% Create GIF
fprintf('Creating GIF from %d saved frames...\n', savedFrameCount);
createGIFFast(imgDir, 'ur5e_auto_obstacle_avoidance.gif');

%% Final Results
finalEEPos = allEEPositions(:, end);
finalDistToPlace = norm(finalEEPos - placePosition(1:3)');

fprintf('\n=== AUTOMATIC OBSTACLE AVOIDANCE RESULTS ===\n');
fprintf('Total computation time: %.2f seconds\n', totalTime);
fprintf('Automatic waypoints generated: %d\n', size(automaticWaypoints, 1));
fprintf('Frames processed: %d\n', totalFrames);
fprintf('Final EE position error: %.4f m\n', finalDistToPlace);

if finalDistToPlace < placeThreshold
    fprintf('✓ Robot successfully reached placement zone\n');
else
    fprintf('✗ Robot did NOT reach placement target\n');
end

if ~objectAttached
    objectPlaceError = norm(objectPosition - placePosition(1:3)');
    fprintf('Object placement error: %.4f m\n', objectPlaceError);
    if objectPlaceError < 0.15
        fprintf('✓ Object successfully placed near target\n');
    else
        fprintf('✗ Object placed far from target\n');
    end
else
    fprintf('✗ Object still attached to gripper\n');
end

%% Cleanup
rmdir(imgDir, 's');
fprintf('Automatic obstacle avoidance simulation completed!\n');

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function waypoints = generateAutomaticPath(startPos, endPos, obstacles, safetyMargin, clearanceHeight, resolution)
    % Automatically generate waypoints to avoid obstacles
    waypoints = [];
    
    % Strategy: Go up, around, then down
    % 1. Lift above obstacles
    liftPos = [startPos(1), startPos(2), clearanceHeight];
    
    % 2. Move horizontally to avoid obstacles
    midX = (startPos(1) + endPos(1)) / 2;
    
    % Find safe Y position (to the side of obstacles)
    safeY = findSafeYPosition(obstacles, safetyMargin);
    navPos = [midX, safeY, clearanceHeight];
    
    % 3. Move above end position
    approachPos = [endPos(1), endPos(2), clearanceHeight];
    
    % Check if waypoints are collision-free and adjust if needed
    candidateWaypoints = [liftPos; navPos; approachPos];
    
    for i = 1:size(candidateWaypoints, 1)
        wp = candidateWaypoints(i, :);
        if ~isPointInObstacles(wp, obstacles, safetyMargin)
            waypoints = [waypoints; wp];
        else
            % Adjust waypoint if it's too close to obstacles
            adjustedWP = adjustWaypointAwayFromObstacles(wp, obstacles, safetyMargin);
            waypoints = [waypoints; adjustedWP];
        end
    end
    
    fprintf('Automatic path planning generated %d waypoints\n', size(waypoints, 1));
end

function safeY = findSafeYPosition(obstacles, safetyMargin)
    % Find a Y position that's safe from all obstacles
    minY = -0.6;
    maxY = 0.6;
    
    % Try different Y positions
    for testY = minY:0.05:maxY
        testPoint = [0.5, testY, 0.4]; % Test at a reasonable X and Z
        if ~isPointInObstacles(testPoint, obstacles, safetyMargin)
            safeY = testY;
            return;
        end
    end
    
    % Fallback: use extreme position
    safeY = maxY;
end

function adjustedWP = adjustWaypointAwayFromObstacles(waypoint, obstacles, safetyMargin)
    % Adjust a waypoint to move it away from obstacles
    adjustedWP = waypoint;
    
    for obsIdx = 1:size(obstacles, 1)
        obs = obstacles(obsIdx, :);
        obsCenter = obs(1:3);
        obsSize = obs(4:6);
        
        % Check if waypoint is too close to this obstacle
        distance = norm(waypoint - obsCenter);
        minSafeDistance = norm(obsSize/2) + safetyMargin;
        
        if distance < minSafeDistance
            % Move waypoint away from obstacle center
            direction = (waypoint - obsCenter) / norm(waypoint - obsCenter);
            adjustedWP = obsCenter + direction * (minSafeDistance + 0.1);
        end
    end
end

function inObstacle = isPointInObstacles(point, obstacles, safetyMargin)
    % Check if a point is inside any obstacle (with safety margin)
    inObstacle = false;
    
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        obsCenter = obs(1:3);
        obsSize = obs(4:6) + 2 * safetyMargin;
        
        withinX = abs(point(1) - obsCenter(1)) <= obsSize(1)/2;
        withinY = abs(point(2) - obsCenter(2)) <= obsSize(2)/2;
        withinZ = abs(point(3) - obsCenter(3)) <= obsSize(3)/2;
        
        if withinX && withinY && withinZ
            inObstacle = true;
            return;
        end
    end
end

function [config, success] = computeRobustIK(ikSolver, robot, pose, initialGuess, maxAttempts)
    % Robust IK computation with multiple strategies
    success = false;
    config = initialGuess(:);
    
    T_target = eye(4);
    T_target(1:3, 4) = pose(1:3);
    T_target(1:3, 1:3) = eul2rotm(pose(4:6), 'XYZ');
    
    % Multiple weight sets and initial guesses
    weightSets = {
        [2, 2, 2, 0.1, 0.1, 0.1],
        [1, 1, 1, 0.3, 0.3, 0.3],
        [1, 1, 1, 0.1, 0.1, 0.1],
        [0.8, 0.8, 0.8, 0.5, 0.5, 0.5]
    };
    
    initialGuesses = {
        initialGuess(:),
        [0; -pi/4; pi/2; -pi/4; pi/2; 0],
        [0; -pi/6; pi/3; -pi/3; pi/2; 0],
        [pi/6; -pi/3; pi/4; -pi/2; pi/2; pi/6]
    };
    
    bestConfig = initialGuess(:);
    bestError = inf;
    
    for attempt = 1:maxAttempts
        try
            weightIdx = mod(attempt-1, length(weightSets)) + 1;
            weights = weightSets{weightIdx};
            
            guessIdx = mod(attempt-1, length(initialGuesses)) + 1;
            currentGuess = initialGuesses{guessIdx};
            
            % Add small random perturbation for later attempts
            if attempt > length(initialGuesses)
                randomOffset = 0.03 * randn(6, 1);
                currentGuess = currentGuess + randomOffset;
            end
            
            % Ensure joint limits
            currentGuess = max(currentGuess, [-2*pi; -pi; -pi; -2*pi; -2*pi; -2*pi]);
            currentGuess = min(currentGuess, [2*pi; pi; pi; 2*pi; 2*pi; 2*pi]);
            
            [config_temp, solInfo] = ikSolver('tool0', T_target, weights, currentGuess);
            
            if solInfo.ExitFlag > 0
                config_temp = config_temp(:);
                if isValidJointConfig(config_temp)
                    T_result = getTransform(robot, config_temp, 'tool0', 'base_link');
                    eePos = T_result(1:3, 4)';
                    posError = norm(eePos - pose(1:3));
                    
                    if posError < bestError
                        bestConfig = config_temp;
                        bestError = posError;
                    end
                    
                    if posError < 0.03  % 3cm tolerance
                        config = config_temp;
                        success = true;
                        return;
                    end
                end
            end
        catch
            % Continue to next attempt
        end
    end
    
    % Use best found solution if any
    if bestError < 0.05  % 5cm tolerance
        config = bestConfig;
        success = true;
    end
end

function valid = isValidJointConfig(config)
    % Check if joint configuration is within reasonable limits
    jointLimits = [
        -2*pi, 2*pi;      % Joint 1
        -pi, pi;          % Joint 2
        -pi, pi;          % Joint 3  
        -2*pi, 2*pi;      % Joint 4
        -2*pi, 2*pi;      % Joint 5
        -2*pi, 2*pi       % Joint 6
    ];
    
    valid = true;
    for i = 1:length(config)
        if config(i) < jointLimits(i, 1) || config(i) > jointLimits(i, 2)
            valid = false;
            return;
        end
    end
end

function jointPath = generateSmoothJointPath(startConfig, endConfig, numWaypoints)
    % Generate smooth joint path with angle wrapping
    startConfig = startConfig(:);
    endConfig = endConfig(:);
    
    if length(startConfig) ~= length(endConfig)
        error('Configuration dimension mismatch');
    end
    
    % Angle wrapping
    jointDiff = endConfig - startConfig;
    jointDiff(jointDiff > pi) = jointDiff(jointDiff > pi) - 2*pi;
    jointDiff(jointDiff < -pi) = jointDiff(jointDiff < -pi) + 2*pi;
    
    % Smooth interpolation
    t = linspace(0, 1, numWaypoints);
    jointPath = startConfig + jointDiff * t;
end

function update2x2MultiView(robot, currentConfig, eePos, pathTrace, ...
                           objectAttached, objectPosition, objectSize, objectColor, ...
                           pickupPos, placePos, automaticWaypoints, ...
                           obstacles, obstacleColors, taskName, ...
                           frameNum, taskIdx, workspaceX, workspaceY, workspaceZ)
    % 2x2 Multi-view visualization with explicit subplot creation
    
    % Clear figure completely
    clf;
    
    % Define views for each subplot position
    viewSettings = {
        [45, 30],    % Top-left (1): Isometric view
        [0, 90],     % Top-right (2): XY view (top-down)
        [90, 0],     % Bottom-left (3): YZ view (side)
        [0, 0]       % Bottom-right (4): ZX view (front)
    };
    
    viewTitles = {
        'Isometric View', 
        'XY View (Top-Down)', 
        'YZ View (Side)', 
        'ZX View (Front)'
    };
    
    % Create each subplot explicitly
    for subplotIdx = 1:4
        % Create subplot explicitly with position
        ax = subplot(2, 2, subplotIdx);
        
        % Clear and set up the axes
        cla(ax);
        hold(ax, 'on');
        
        % Show robot in current subplot
        show(robot, currentConfig, 'Frames', 'off', 'PreservePlot', false, 'Visuals', 'on', 'Parent', ax);
        
        % Set workspace limits immediately
        set(ax, 'XLim', workspaceX, 'YLim', workspaceY, 'ZLim', workspaceZ);
        
        % Ground plane
        patch(ax, [-0.8 0.8 0.8 -0.8], [-0.8 -0.8 0.8 0.8], [0 0 0 0], [0.9, 0.9, 0.9], ...
              'FaceAlpha', 0.2, 'EdgeColor', 'none');
        
        % Draw obstacles in the same axes
        for obsIdx = 1:size(obstacles, 1)
            obs = obstacles(obsIdx, :);
            obsColor = obstacleColors(obsIdx, :);
            plotObstacleInAxes(ax, obs(1:3), obs(4:6), obsColor);
        end
        
        % Object position
        plotBoxInAxes(ax, objectPosition, objectSize, objectColor);
        
        % Path trace (sampled for performance)
        if size(pathTrace, 2) > 1
            sampleIdx = 1:max(1, floor(size(pathTrace, 2)/30)):size(pathTrace, 2);
            plot3(ax, pathTrace(1, sampleIdx), pathTrace(2, sampleIdx), pathTrace(3, sampleIdx), ...
                  'g-', 'LineWidth', 3);
        end
        
        % Target markers
        plot3(ax, pickupPos(1), pickupPos(2), pickupPos(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', [0, 0.8, 0], 'MarkerEdgeColor', 'black');
        plot3(ax, placePos(1), placePos(2), placePos(3), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', [0, 0.4, 1], 'MarkerEdgeColor', 'black');
        
        % Target labels (only for isometric view to avoid clutter)
        if subplotIdx == 1
            text(ax, pickupPos(1), pickupPos(2), pickupPos(3) + 0.15, 'PICKUP', ...
                 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0, 0.6, 0]);
            text(ax, placePos(1), placePos(2), placePos(3) + 0.15, 'PLACE', ...
                 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0, 0.3, 0.8]);
        end
        
        % Automatic waypoints
        for wpIdx = 1:size(automaticWaypoints, 1)
            wp = automaticWaypoints(wpIdx, :);
            plot3(ax, wp(1), wp(2), wp(3), 'md', 'MarkerSize', 8, 'MarkerFaceColor', [1, 0, 1], 'MarkerEdgeColor', 'black');
        end
        
        % Current end-effector
        plot3(ax, eePos(1), eePos(2), eePos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
        
        % Object attachment indicator (only in isometric view)
        if objectAttached && subplotIdx == 1
            text(ax, eePos(1), eePos(2), eePos(3) + 0.2, 'ATTACHED', ...
                 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold', ...
                 'Color', 'red', 'BackgroundColor', 'yellow');
            
            % Connection line
            plot3(ax, [eePos(1), objectPosition(1)], [eePos(2), objectPosition(2)], ...
                  [eePos(3), objectPosition(3)], 'r--', 'LineWidth', 2);
        end
        
        % Distance line to current target (only in isometric view)
        if subplotIdx == 1
            if taskIdx <= 2
                targetPos = pickupPos(1:3);
                lineColor = [0, 0.8, 0];
            else
                targetPos = placePos(1:3);
                lineColor = [0, 0.4, 1];
            end
            plot3(ax, [eePos(1), targetPos(1)], [eePos(2), targetPos(2)], [eePos(3), targetPos(3)], ...
                  '--', 'Color', lineColor, 'LineWidth', 2);
        end
        
        % Lighting
        lighting(ax, 'gouraud');
        light(ax, 'Position', [1, 1, 1], 'Style', 'infinite');
        light(ax, 'Position', [-1, -1, 1], 'Style', 'infinite');
        
        % Set view angle
        view(ax, viewSettings{subplotIdx});
        
        % Set axis properties
        axis(ax, 'equal'); 
        grid(ax, 'on');
        set(ax, 'GridAlpha', 0.3);
        
        % Force the limits again
        set(ax, 'XLim', workspaceX, 'YLim', workspaceY, 'ZLim', workspaceZ);
        
        % Subplot title
        title(ax, viewTitles{subplotIdx}, 'FontSize', 12, 'FontWeight', 'bold');
        
        % Axis labels for all subplots
        xlabel(ax, 'X (m)', 'FontSize', 10);
        ylabel(ax, 'Y (m)', 'FontSize', 10);
        zlabel(ax, 'Z (m)', 'FontSize', 10);
        
        % Set font size
        set(ax, 'FontSize', 9);
        
        hold(ax, 'off');
    end
    
    % Main title for entire figure
    sgtitle(sprintf('UR5e Auto Obstacle Avoidance: %s | Frame: %d | EE: [%.2f, %.2f, %.2f] | Task: %d/6', ...
                   taskName, frameNum, eePos, taskIdx), 'FontSize', 14, 'FontWeight', 'bold');
    
    % Ensure figure is properly drawn
    drawnow;
end

function plotObstacleInAxes(ax, center, size, color)
    % Plot 3D box obstacle in specified axes
    x = center(1) + size(1)/2 * [-1, 1, 1, -1, -1, 1, 1, -1];
    y = center(2) + size(2)/2 * [-1, -1, 1, 1, -1, -1, 1, 1];
    z = center(3) + size(3)/2 * [-1, -1, -1, -1, 1, 1, 1, 1];
    
    vertices = [x; y; z]';
    faces = [1,2,6,5; 3,4,8,7; 1,2,3,4; 5,6,7,8; 1,4,8,5; 2,3,7,6];
    
    patch(ax, 'Vertices', vertices, 'Faces', faces, 'FaceColor', color, ...
          'FaceAlpha', 0.8, 'EdgeColor', 'black', 'LineWidth', 1.5);
end

function plotBoxInAxes(ax, position, size, color)
    % Plot object box in specified axes
    x = position(1) + size(1)/2 * [-1, 1, 1, -1, -1, 1, 1, -1];
    y = position(2) + size(2)/2 * [-1, -1, 1, 1, -1, -1, 1, 1];
    z = position(3) + size(3)/2 * [-1, -1, -1, -1, 1, 1, 1, 1];
    
    vertices = [x; y; z]';
    faces = [1,2,6,5; 3,4,8,7; 1,2,3,4; 5,6,7,8; 1,4,8,5; 2,3,7,6];
    
    patch(ax, 'Vertices', vertices, 'Faces', faces, 'FaceColor', color, ...
          'FaceAlpha', 0.9, 'EdgeColor', 'black', 'LineWidth', 1);
end

function createGIFFast(imgDir, gifFilename)
    % Fast GIF creation
    imageFiles = dir(fullfile(imgDir, '*.png'));
    if isempty(imageFiles), return; end
    
    % Sort files numerically
    fileNumbers = zeros(length(imageFiles), 1);
    for i = 1:length(imageFiles)
        fileNumbers(i) = sscanf(imageFiles(i).name, 'frame_%d.png');
    end
    [~, sortIdx] = sort(fileNumbers);
    imageFiles = imageFiles(sortIdx);
    
    % Create GIF
    for i = 1:length(imageFiles)
        filename = fullfile(imgDir, imageFiles(i).name);
        img = imread(filename);
        [imgIndexed, cmap] = rgb2ind(img, 128);
        
        if i == 1
            imwrite(imgIndexed, cmap, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
        else
            imwrite(imgIndexed, cmap, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        end
    end
    
    fprintf('Automatic obstacle avoidance GIF saved: %s\n', gifFilename);
    end9