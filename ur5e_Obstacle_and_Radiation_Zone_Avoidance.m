%% UR5e Robot Simulation with Automatic Obstacle and Radiation Zone Avoidance
% Enhanced with spherical radiation zone avoidance capability

clear; clc; close all;

%% ========================================================================
%% USER CONFIGURABLE PARAMETERS
%% ========================================================================

% ROBOT INITIAL CONFIGURATION
verticalConfig = [0; 0; 0; 0; 0; 0];  % [Joint1; Joint2; Joint3; Joint4; Joint5; Joint6] in radians

% TARGET POSITIONS [X, Y, Z, RotX, RotY, RotZ]
pickupPosition = [0.5, 0.3, 0.2, pi, 0, 0];     % Pickup target location
placePosition = [0.6, -0.4, 0.2, pi, 0, 0];    % Place target location

% RECTANGULAR OBSTACLE PARAMETERS
obstacles = [
    % [X_center, Y_center, Z_center, X_size, Y_size, Z_size]
    [0.5, -0.05, 0.25, 0.12, 0.3, 0.5];   % Main central obstacle
    [0.6, -0.25, 0.12, 0.08, 0.08, 0.24]; % Small side obstacle
];
obstacleColors = [
    [0.8, 0.2, 0.2];  % Red for main obstacle
    [0.6, 0.1, 0.1];  % Dark red for side obstacle
];

% SPHERICAL RADIATION ZONES
% [X_center, Y_center, Z_center, radius]
radiationZones = [
    [0.65, 0, 0.65, 0.1];  % Spherical radiation zone
];
radiationColors = [
    [1.0, 1.0, 0.0];  % Yellow for radiation zone
];

safetyMargin = 0.06;  % Safety margin around obstacles (meters)
radiationSafetyMargin = 0.08;  % Extra safety margin for radiation zones

% AUTOMATIC PATH PLANNING PARAMETERS
clearanceHeight = 0.8;     % Height to clear obstacles automatically (increased for radiation zone)
pathResolution = 0.03;     % Path planning resolution (meters) - finer for better avoidance
maxPlanningAttempts = 100; % Maximum attempts for automatic planning (increased)

% OBJECT PROPERTIES
objectSize = [0.05, 0.05, 0.05];                % Object dimensions [length, width, height]
objectColor = [1.0, 0.5, 0.0];                  % Object color [R, G, B] (orange)

% PATH PLANNING PARAMETERS
numWaypoints = 25;       % Number of waypoints per path segment (increased)
numSteps = 6;            % Interpolation steps between waypoints
pickupThreshold = 0.10;  % Distance threshold for pickup (meters)
placeThreshold = 0.12;   % Distance threshold for placement (meters)
maxIKAttempts = 30;      % Maximum IK solving attempts per waypoint (increased)

% VISUALIZATION PARAMETERS
updateEvery = 4;         % Update visualization every N frames
saveEvery = 2;           % Save image every N frames
imageQuality = 1.0;      % Image size factor (0.6 = 60% of original size)
pauseTime = 0.02;        % Pause between frames (seconds)

% WORKSPACE LIMITS
workspaceX = [-1.0, 1.0];    % X-axis limits for visualization
workspaceY = [-1.0, 1.0];    % Y-axis limits for visualization  
workspaceZ = [-0.25, 1.25];    % Z-axis limits for visualization

%% ========================================================================
%% SIMULATION START
%% ========================================================================

fprintf('=== UR5e Automatic Obstacle and Radiation Zone Avoidance Simulation ===\n');
fprintf('Pickup Position: [%.2f, %.2f, %.2f]\n', pickupPosition(1:3));
fprintf('Place Position:  [%.2f, %.2f, %.2f]\n', placePosition(1:3));
fprintf('Number of Obstacles: %d\n', size(obstacles, 1));
fprintf('Number of Radiation Zones: %d\n', size(radiationZones, 1));
fprintf('Safety Margin: %.3f m\n', safetyMargin);
fprintf('Radiation Safety Margin: %.3f m\n', radiationSafetyMargin);
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

%% Enhanced Automatic Path Planning with Radiation Zone Avoidance
fprintf('Planning automatic path with radiation zone avoidance...\n');

% Generate enhanced automatic waypoints
automaticWaypoints = generateEnhancedAutomaticPath(pickupPosition(1:3), placePosition(1:3), ...
                                                 obstacles, radiationZones, safetyMargin, ...
                                                 radiationSafetyMargin, clearanceHeight, pathResolution);

% Validate automatic waypoints
if isempty(automaticWaypoints)
    fprintf('Warning: No automatic waypoints generated, creating simple fallback path\n');
    automaticWaypoints = [
        pickupPosition(1), pickupPosition(2), clearanceHeight;
        placePosition(1), placePosition(2), clearanceHeight
    ];
end

% Ensure automaticWaypoints is a proper matrix
if size(automaticWaypoints, 2) ~= 3
    fprintf('Error: automaticWaypoints has wrong dimensions (%dx%d), reshaping...\n', size(automaticWaypoints, 1), size(automaticWaypoints, 2));
    if numel(automaticWaypoints) >= 3
        automaticWaypoints = reshape(automaticWaypoints(1:end), [], 3);
    else
        automaticWaypoints = [placePosition(1), placePosition(2), clearanceHeight];
    end
end

fprintf('Generated %d automatic waypoints with radiation zone avoidance\n', size(automaticWaypoints, 1));

% Display waypoints for verification
for i = 1:size(automaticWaypoints, 1)
    fprintf('  Auto waypoint %d: [%.3f, %.3f, %.3f]\n', i, automaticWaypoints(i, 1), automaticWaypoints(i, 2), automaticWaypoints(i, 3));
end

% Create complete path sequence with validation
try
    pathSequence = [
        verticalGripperPos';           % Start position
        pickupPosition(1:3);          % Pickup position
        automaticWaypoints;           % Automatically generated waypoints
        placePosition(1:3);           % Final place position
    ];
catch ME
    fprintf('Error creating path sequence: %s\n', ME.message);
    fprintf('automaticWaypoints size: %dx%d\n', size(automaticWaypoints, 1), size(automaticWaypoints, 2));
    fprintf('verticalGripperPos size: %dx%d\n', size(verticalGripperPos, 1), size(verticalGripperPos, 2));
    
    % Emergency fallback path
    pathSequence = [
        verticalGripperPos';
        pickupPosition(1:3);
        [pickupPosition(1), pickupPosition(2), clearanceHeight];
        [placePosition(1), placePosition(2), clearanceHeight];
        placePosition(1:3);
    ];
    
    % Update automaticWaypoints to match
    automaticWaypoints = [
        [pickupPosition(1), pickupPosition(2), clearanceHeight];
        [placePosition(1), placePosition(2), clearanceHeight]
    ];
end

% Final validation of path sequence
if size(pathSequence, 2) ~= 3
    error('Path sequence has wrong dimensions: %dx%d (expected Nx3)', size(pathSequence, 1), size(pathSequence, 2));
end

fprintf('Complete path sequence created with %d waypoints\n', size(pathSequence, 1));

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

fprintf('Computing IK solutions for enhanced automatic path...\n');
numWaypoints_total = size(pathSequence, 1);

% Validate path sequence
if isempty(pathSequence) || size(pathSequence, 2) ~= 3
    error('Invalid path sequence generated. Size: %dx%d (expected Nx3)', size(pathSequence, 1), size(pathSequence, 2));
end

fprintf('Path sequence has %d waypoints\n', numWaypoints_total);
for i = 1:numWaypoints_total
    fprintf('  Waypoint %d: [%.3f, %.3f, %.3f]\n', i, pathSequence(i, 1), pathSequence(i, 2), pathSequence(i, 3));
end

waypointConfigs = cell(numWaypoints_total, 1);
waypointConfigs{1} = verticalConfig;

for i = 2:numWaypoints_total
    if i > size(pathSequence, 1)
        error('Waypoint index %d exceeds path sequence size %d', i, size(pathSequence, 1));
    end
    
    targetPos = pathSequence(i, :);
    
    % Validate target position
    if length(targetPos) ~= 3 || any(isnan(targetPos)) || any(isinf(targetPos))
        fprintf('Warning: Invalid target position at waypoint %d: [%.3f, %.3f, %.3f]\n', i, targetPos);
        % Use previous position slightly modified
        if i > 1
            targetPos = pathSequence(i-1, :) + [0.1, 0, 0];
        else
            targetPos = [0.5, 0.3, 0.2]; % Default safe position
        end
        fprintf('Using corrected position: [%.3f, %.3f, %.3f]\n', targetPos);
    end
    
    targetPose = [targetPos, pi, 0, 0]; % Keep gripper pointing down
    
    fprintf('Computing IK for waypoint %d: [%.3f, %.3f, %.3f]\n', i, targetPos);
    
    % Use previous configuration as initial guess
    if i > 1 && ~isempty(waypointConfigs{i-1})
        initialGuess = waypointConfigs{i-1};
    else
        initialGuess = verticalConfig;
    end
    
    % Compute robust IK with additional error handling
    try
        [config, success] = computeRobustIK(ikSolver, robot, targetPose, initialGuess, maxIKAttempts);
        
        if success
            waypointConfigs{i} = config;
            T_result = getTransform(robot, config, 'tool0', 'base_link');
            actualPos = T_result(1:3, 4);
            error = norm(actualPos - targetPos');
            fprintf('  ✓ IK solved successfully, error: %.4f m\n', error);
        else
            % Try alternative approach with relaxed constraints
            fprintf('  Standard IK failed, trying alternative approach...\n');
            
            % Try with a modified target (slightly lower Z)
            altTargetPose = [targetPos(1), targetPos(2), targetPos(3) - 0.05, pi, 0, 0];
            [config, success] = computeRobustIK(ikSolver, robot, altTargetPose, initialGuess, maxIKAttempts);
            
            if success
                waypointConfigs{i} = config;
                fprintf('  ✓ Alternative IK solved successfully\n');
            else
                % Last resort: use previous configuration with small modification
                fprintf('  Using fallback configuration (modified previous)\n');
                if i > 1 && ~isempty(waypointConfigs{i-1})
                    fallbackConfig = waypointConfigs{i-1} + 0.02 * randn(6, 1);
                    % Ensure within joint limits
                    fallbackConfig = max(fallbackConfig, [-2*pi; -pi; -pi; -2*pi; -2*pi; -2*pi]);
                    fallbackConfig = min(fallbackConfig, [2*pi; pi; pi; 2*pi; 2*pi; 2*pi]);
                    waypointConfigs{i} = fallbackConfig;
                else
                    waypointConfigs{i} = verticalConfig;
                end
            end
        end
        
    catch ME
        fprintf('  ERROR in IK computation: %s\n', ME.message);
        fprintf('  Using safe fallback configuration\n');
        
        % Emergency fallback
        if i > 1 && ~isempty(waypointConfigs{i-1})
            waypointConfigs{i} = waypointConfigs{i-1};
        else
            waypointConfigs{i} = verticalConfig;
        end
    end
    
    % Validate the computed configuration
    if isempty(waypointConfigs{i}) || length(waypointConfigs{i}) ~= 6
        fprintf('  Configuration validation failed, using safe default\n');
        waypointConfigs{i} = verticalConfig;
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
imgDir = 'ur5e_radiation_avoidance_frames';
if exist(imgDir, 'dir'), rmdir(imgDir, 's'); end
mkdir(imgDir);

% Create figure with explicit size and properties for 2x2 layout
fig = figure('Name', 'UR5e Radiation Zone Avoidance - 2x2 Multi-View', ...
             'Position', [50, 50, 1600, 1200], ...
             'Renderer', 'opengl', ...
             'Color', 'white', ...
             'WindowState', 'maximized', ...
             'NumberTitle', 'off');

% Test the subplot layout
fprintf('Testing 2x2 subplot layout...\n');
for i = 1:4
    subplot(2, 2, i);
    text(0.5, 0.5, sprintf('View %d', i), 'HorizontalAlignment', 'center', ...
         'FontSize', 16, 'FontWeight', 'bold');
    title(sprintf('Subplot %d', i));
end
pause(1);
fprintf('2x2 layout test completed. All 4 subplots should be visible.\n');

%% Execution Loop
fprintf('Starting radiation zone avoidance simulation...\n');
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
    
    % Enhanced 2x2 Multi-view visualization with radiation zones
    if mod(frameIdx, updateEvery) == 0 || frameIdx == 1 || frameIdx == totalFrames
        figure(fig);
        
        updateEnhanced2x2MultiView(robot, currentConfig, eePos, allEEPositions(:, 1:frameIdx), ...
                          objectAttached, currentObjectPos, objectSize, objectColor, ...
                          pickupPosition, placePosition, automaticWaypoints, ...
                          obstacles, obstacleColors, radiationZones, radiationColors, ...
                          pathNames{currentTask}, frameIdx, currentTask, ...
                          workspaceX, workspaceY, workspaceZ);
        
        % Save frame
        if mod(frameIdx, saveEvery) == 0
            savedFrameCount = savedFrameCount + 1;
            filename = fullfile(imgDir, sprintf('frame_%04d.png', savedFrameCount));
            frame = getframe(fig);
            img = imresize(frame.cdata, imageQuality);
            imwrite(img, filename, 'png', 'Compression', 'high');
        end
        
        drawnow;
        pause(pauseTime);
    end
end

executionTime = toc;
totalTime = precomputeTime + executionTime;

%% Create GIF
fprintf('Creating GIF from %d saved frames...\n', savedFrameCount);
createGIFFast(imgDir, 'ur5e_radiation_zone_avoidance.gif');

%% Final Results
finalEEPos = allEEPositions(:, end);
finalDistToPlace = norm(finalEEPos - placePosition(1:3)');

fprintf('\n=== RADIATION ZONE AVOIDANCE RESULTS ===\n');
fprintf('Total computation time: %.2f seconds\n', totalTime);
fprintf('Automatic waypoints generated: %d\n', size(automaticWaypoints, 1));
fprintf('Frames processed: %d\n', totalFrames);
fprintf('Final EE position error: %.4f m\n', finalDistToPlace);

% Check radiation zone avoidance
radiationViolations = 0;
for i = 1:totalFrames
    if isPointInRadiationZones(allEEPositions(:, i)', radiationZones, radiationSafetyMargin)
        radiationViolations = radiationViolations + 1;
    end
end

fprintf('Radiation zone violations: %d/%d frames (%.2f%%)\n', ...
        radiationViolations, totalFrames, 100*radiationViolations/totalFrames);

if radiationViolations == 0
    fprintf('✓ Successfully avoided all radiation zones\n');
else
    fprintf('✗ Path entered radiation zones %d times\n', radiationViolations);
end

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
fprintf('Radiation zone avoidance simulation completed!\n');

%% ========================================================================
%% ENHANCED HELPER FUNCTIONS
%% ========================================================================

function waypoints = generateEnhancedAutomaticPath(startPos, endPos, obstacles, radiationZones, ...
                                                  safetyMargin, radiationSafetyMargin, clearanceHeight, resolution)
    % Enhanced automatic path generation with radiation zone avoidance
    fprintf('Generating enhanced path with radiation zone avoidance...\n');
    
    waypoints = [];
    
    % Strategy: Multi-level approach to avoid both rectangular and spherical obstacles
    
    % 1. Initial lift to clearance height
    liftPos = [startPos(1), startPos(2), clearanceHeight];
    
    % 2. Check if direct path is possible at clearance height
    directPath = [liftPos; [endPos(1), endPos(2), clearanceHeight]];
    
    if isPathClear(directPath, obstacles, radiationZones, safetyMargin, radiationSafetyMargin)
        fprintf('Direct path at clearance height is clear\n');
        waypoints = [endPos(1), endPos(2), clearanceHeight];
    else
        fprintf('Direct path blocked, computing alternative route...\n');
        
        % 3. Find safe intermediate positions using robust approach
        intermediateWaypoints = findSafePathAroundObstacles(liftPos, [endPos(1), endPos(2), clearanceHeight], ...
                                                           obstacles, radiationZones, safetyMargin, radiationSafetyMargin);
        
        % If still blocked, try higher altitude
        if isempty(intermediateWaypoints)
            higherClearance = clearanceHeight + 0.3;
            fprintf('Trying higher clearance: %.2f m\n', higherClearance);
            
            liftPos = [startPos(1), startPos(2), higherClearance];
            intermediateWaypoints = findSafePathAroundObstacles(liftPos, [endPos(1), endPos(2), higherClearance], ...
                                                               obstacles, radiationZones, safetyMargin, radiationSafetyMargin);
        end
        
        % If still empty, try even higher
        if isempty(intermediateWaypoints)
            veryHighClearance = clearanceHeight + 0.6;
            fprintf('Trying very high clearance: %.2f m\n', veryHighClearance);
            
            liftPos = [startPos(1), startPos(2), veryHighClearance];
            intermediateWaypoints = findSafePathAroundObstacles(liftPos, [endPos(1), endPos(2), veryHighClearance], ...
                                                               obstacles, radiationZones, safetyMargin, radiationSafetyMargin);
        end
        
        waypoints = intermediateWaypoints;
    end
    
    % Ensure we have at least one waypoint - this is critical!
    if isempty(waypoints)
        % Emergency fallback: Simple safe path that avoids all obstacles
        emergencyHeight = max(clearanceHeight + 0.8, 1.2);
        fprintf('EMERGENCY: Using very high-altitude path at %.2f m\n', emergencyHeight);
        
        % Create a simple 3-point path that goes around the radiation zone
        midX = (startPos(1) + endPos(1)) / 2;
        
        % Move to safe Y position (away from obstacles and radiation zone)
        safeY = findSafeYPosition(obstacles, radiationZones, safetyMargin, radiationSafetyMargin);
        
        waypoints = [
            startPos(1), startPos(2), emergencyHeight;        % Lift straight up
            midX, safeY, emergencyHeight;                     % Move to safe intermediate point
            endPos(1), endPos(2), emergencyHeight            % Move above target
        ];
    end
    
    % Ensure waypoints is always a matrix, not empty
    if isempty(waypoints)
        % Absolute fallback - very simple high path
        waypoints = [
            startPos(1), startPos(2), 1.5;
            endPos(1), endPos(2), 1.5
        ];
        fprintf('ABSOLUTE FALLBACK: Using maximum height path\n');
    end
    
    fprintf('Generated %d waypoints for enhanced path\n', size(waypoints, 1));
    
    % Validate waypoints
    if size(waypoints, 2) ~= 3
        error('Generated waypoints have wrong dimensions: %dx%d (expected Nx3)', size(waypoints, 1), size(waypoints, 2));
    end
end

function safeWaypoints = findSafePathAroundObstacles(startPos, endPos, obstacles, radiationZones, ...
                                                    safetyMargin, radiationSafetyMargin)
    % Find safe path around obstacles using geometric planning
    safeWaypoints = [];
    
    try
        % Generate candidate intermediate points
        candidates = generateCandidateWaypoints(startPos, endPos, obstacles, radiationZones, ...
                                              safetyMargin, radiationSafetyMargin);
        
        if isempty(candidates)
            fprintf('No safe candidates found, creating simple detour\n');
            % Create a simple detour around radiation zone
            midPoint = (startPos + endPos) / 2;
            midPoint(2) = midPoint(2) + 0.4; % Move to side
            safeWaypoints = midPoint;
            return;
        end
        
        % Select best waypoints (simple greedy approach)
        currentPos = startPos;
        maxIterations = 10; % Prevent infinite loops
        iteration = 0;
        
        while norm(currentPos - endPos) > 0.15 && iteration < maxIterations
            iteration = iteration + 1;
            bestCandidate = [];
            bestScore = inf;
            
            for i = 1:size(candidates, 1)
                candidate = candidates(i, :);
                
                % Check if path from current to candidate is clear
                testPath = [currentPos; candidate];
                if isPathClear(testPath, obstacles, radiationZones, safetyMargin, radiationSafetyMargin)
                    % Score based on distance to goal and distance from obstacles
                    distToGoal = norm(candidate - endPos);
                    distFromObstacles = minDistanceToObstacles(candidate, obstacles, radiationZones);
                    score = distToGoal - 0.3 * distFromObstacles; % Prefer closer to goal but farther from obstacles
                    
                    if score < bestScore
                        bestScore = score;
                        bestCandidate = candidate;
                    end
                end
            end
            
            if ~isempty(bestCandidate)
                safeWaypoints = [safeWaypoints; bestCandidate];
                currentPos = bestCandidate;
                
                % Remove used candidate to avoid revisiting
                candidateRow = find(ismember(candidates, bestCandidate, 'rows'), 1);
                if ~isempty(candidateRow)
                    candidates(candidateRow, :) = [];
                end
            else
                break; % No safe path found
            end
        end
        
        % If we couldn't reach the goal, add a final waypoint closer to target
        if norm(currentPos - endPos) > 0.15 && ~isempty(safeWaypoints)
            % Try to add a waypoint closer to the end
            finalWaypoint = endPos + 0.1 * (currentPos - endPos) / norm(currentPos - endPos);
            finalWaypoint(3) = endPos(3); % Keep same height
            
            if ~isPointInObstacles(finalWaypoint, obstacles, safetyMargin) && ...
               ~isPointInRadiationZones(finalWaypoint, radiationZones, radiationSafetyMargin)
                safeWaypoints = [safeWaypoints; finalWaypoint];
            end
        end
        
    catch ME
        fprintf('Error in findSafePathAroundObstacles: %s\n', ME.message);
        % Fallback: simple side path
        midPoint = (startPos + endPos) / 2;
        midPoint(2) = midPoint(2) + 0.5; % Move to side
        safeWaypoints = midPoint;
    end
end

function candidates = generateCandidateWaypoints(startPos, endPos, obstacles, radiationZones, ...
                                               safetyMargin, radiationSafetyMargin)
    % Generate candidate waypoints around obstacles
    candidates = [];
    
    % Grid-based candidate generation
    xRange = [min(startPos(1), endPos(1)) - 0.3, max(startPos(1), endPos(1)) + 0.3];
    yRange = [min(startPos(2), endPos(2)) - 0.5, max(startPos(2), endPos(2)) + 0.5];
    zHeight = startPos(3); % Keep at same height
    
    xGrid = xRange(1):0.1:xRange(2);
    yGrid = yRange(1):0.1:yRange(2);
    
    for x = xGrid
        for y = yGrid
            candidate = [x, y, zHeight];
            
            % Check if candidate is safe
            if ~isPointInObstacles(candidate, obstacles, safetyMargin) && ...
               ~isPointInRadiationZones(candidate, radiationZones, radiationSafetyMargin)
                candidates = [candidates; candidate];
            end
        end
    end
    
    % Add specific waypoints around radiation zones
    for i = 1:size(radiationZones, 1)
        zone = radiationZones(i, :);
        center = zone(1:3);
        radius = zone(4);
        
        % Generate points around the sphere
        angles = 0:pi/4:2*pi;
        for angle = angles
            offset = (radius + radiationSafetyMargin + 0.1) * [cos(angle), sin(angle), 0];
            candidate = center + offset;
            candidate(3) = zHeight; % Keep at planning height
            
            if ~isPointInObstacles(candidate, obstacles, safetyMargin)
                candidates = [candidates; candidate];
            end
        end
    end
end

function clear = isPathClear(path, obstacles, radiationZones, safetyMargin, radiationSafetyMargin)
    % Check if entire path is clear of obstacles and radiation zones
    clear = true;
    
    for i = 1:size(path, 1)-1
        startPoint = path(i, :);
        endPoint = path(i+1, :);
        
        % Sample points along the path segment
        numSamples = ceil(norm(endPoint - startPoint) / 0.05); % 5cm resolution
        for j = 0:numSamples
            t = j / numSamples;
            testPoint = startPoint + t * (endPoint - startPoint);
            
            if isPointInObstacles(testPoint, obstacles, safetyMargin) || ...
               isPointInRadiationZones(testPoint, radiationZones, radiationSafetyMargin)
                clear = false;
                return;
            end
        end
    end
end

function minDist = minDistanceToObstacles(point, obstacles, radiationZones)
    % Find minimum distance to any obstacle or radiation zone
    minDist = inf;
    
    % Check rectangular obstacles
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        obsCenter = obs(1:3);
        obsSize = obs(4:6);
        
        % Distance to box
        dx = max(0, abs(point(1) - obsCenter(1)) - obsSize(1)/2);
        dy = max(0, abs(point(2) - obsCenter(2)) - obsSize(2)/2);
        dz = max(0, abs(point(3) - obsCenter(3)) - obsSize(3)/2);
        dist = sqrt(dx^2 + dy^2 + dz^2);
        
        minDist = min(minDist, dist);
    end
    
    % Check radiation zones (spherical)
    for i = 1:size(radiationZones, 1)
        zone = radiationZones(i, :);
        center = zone(1:3);
        radius = zone(4);
        
        dist = max(0, norm(point - center) - radius);
        minDist = min(minDist, dist);
    end
end

function inRadiation = isPointInRadiationZones(point, radiationZones, safetyMargin)
    % Check if point is inside any spherical radiation zone
    inRadiation = false;
    
    for i = 1:size(radiationZones, 1)
        zone = radiationZones(i, :);
        center = zone(1:3);
        radius = zone(4) + safetyMargin;
        
        distance = norm(point - center);
        if distance <= radius
            inRadiation = true;
            return;
        end
    end
end

function inObstacle = isPointInObstacles(point, obstacles, safetyMargin)
    % Check if a point is inside any rectangular obstacle (with safety margin)
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

function updateEnhanced2x2MultiView(robot, currentConfig, eePos, pathTrace, ...
                           objectAttached, objectPosition, objectSize, objectColor, ...
                           pickupPos, placePos, automaticWaypoints, ...
                           obstacles, obstacleColors, radiationZones, radiationColors, ...
                           taskName, frameNum, taskIdx, workspaceX, workspaceY, workspaceZ)
    % Enhanced 2x2 Multi-view visualization with radiation zones
    
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
        
        % Draw rectangular obstacles
        for obsIdx = 1:size(obstacles, 1)
            obs = obstacles(obsIdx, :);
            obsColor = obstacleColors(obsIdx, :);
            plotObstacleInAxes(ax, obs(1:3), obs(4:6), obsColor);
        end
        
        % Draw spherical radiation zones
        for radIdx = 1:size(radiationZones, 1)
            zone = radiationZones(radIdx, :);
            radColor = radiationColors(radIdx, :);
            plotSphereInAxes(ax, zone(1:3), zone(4), radColor);
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
                 
            % Radiation zone label
            for radIdx = 1:size(radiationZones, 1)
                zone = radiationZones(radIdx, :);
                text(ax, zone(1), zone(2), zone(3) + zone(4) + 0.1, 'RADIATION', ...
                     'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.8, 0.6, 0]);
            end
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
    sgtitle(sprintf('UR5e Radiation Zone Avoidance: %s | Frame: %d | EE: [%.2f, %.2f, %.2f] | Task: %d/6', ...
                   taskName, frameNum, eePos, taskIdx), 'FontSize', 14, 'FontWeight', 'bold');
    
    % Ensure figure is properly drawn
    drawnow;
end

function plotSphereInAxes(ax, center, radius, color)
    % Plot transparent sphere for radiation zone
    [X, Y, Z] = sphere(20);
    X = X * radius + center(1);
    Y = Y * radius + center(2);
    Z = Z * radius + center(3);
    
    surf(ax, X, Y, Z, 'FaceColor', color, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    % Add wireframe for better visibility
    [X_wire, Y_wire, Z_wire] = sphere(10);
    X_wire = X_wire * radius + center(1);
    Y_wire = Y_wire * radius + center(2);
    Z_wire = Z_wire * radius + center(3);
    
    mesh(ax, X_wire, Y_wire, Z_wire, 'FaceColor', 'none', 'EdgeColor', color, 'LineWidth', 1);
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
    
    fprintf('Radiation zone avoidance GIF saved: %s\n', gifFilename);
end