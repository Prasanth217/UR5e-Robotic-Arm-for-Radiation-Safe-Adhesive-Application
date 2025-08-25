%% Fast UR5e Simulation - Optimized with Vectorization
% Fast version using vectorized operations and pre-allocation

clear; clc; close all;

%% Robot Setup
fprintf('Setting up UR5e (fast vectorized version)...\n');

robot = loadrobot('universalUR5e');
robot.DataFormat = 'column';

% Vertical starting configuration
verticalConfig = [0; 0; 0; 0; 0; 0];
currentConfig = verticalConfig;

% Calculate vertical gripper position
T_vertical = getTransform(robot, verticalConfig, 'tool0', 'base_link');
verticalGripperPos = T_vertical(1:3, 4);

%% Define Target Positions
pickupPosition = [0.6, 0.4, 0.2, pi, 0, 0];     % Pickup target
placePosition = [0.8, -0.5, 0.2, pi, 0, 0];    % Place target  

fprintf('Targets: Pickup[%.2f,%.2f,%.2f] Place[%.2f,%.2f,%.2f]\n', ...
        pickupPosition(1:3), placePosition(1:3));

%% Compute Configurations
ikSolver = inverseKinematics('RigidBodyTree', robot);
ikSolver.SolverParameters.AllowRandomRestart = false;

%% Compute Configurations with Multiple Attempts
ikSolver = inverseKinematics('RigidBodyTree', robot);
ikSolver.SolverParameters.AllowRandomRestart = false;

% Compute pickup configuration
pickupConfig = computeIK(ikSolver, pickupPosition, verticalConfig);
T_pickup = getTransform(robot, pickupConfig, 'tool0', 'base_link');
pickupError = norm(T_pickup(1:3, 4) - pickupPosition(1:3)');

% Compute place configuration with multiple attempts for better solution
fprintf('Computing place configuration...\n');
bestPlaceConfig = [];
bestPlaceError = inf;

% Try multiple initial guesses
initialGuesses = {pickupConfig, verticalConfig, [0;0;-pi/2;0;0;0], [0;pi/4;0;-pi/2;0;0]};

for i = 1:length(initialGuesses)
    placeConfig_temp = computeIK(ikSolver, placePosition, initialGuesses{i});
    T_place_temp = getTransform(robot, placeConfig_temp, 'tool0', 'base_link');
    placeError_temp = norm(T_place_temp(1:3, 4) - placePosition(1:3)');
    
    fprintf('  Attempt %d: Error = %.4f m\n', i, placeError_temp);
    
    if placeError_temp < bestPlaceError
        bestPlaceConfig = placeConfig_temp;
        bestPlaceError = placeError_temp;
    end
end

placeConfig = bestPlaceConfig;
T_place = getTransform(robot, placeConfig, 'tool0', 'base_link');

fprintf('FINAL IK Results:\n');
fprintf('  Pickup Error: %.4f m\n', pickupError);
fprintf('  Place Error: %.4f m\n', bestPlaceError);
fprintf('  Actual place position: [%.3f, %.3f, %.3f]\n', T_place(1:3, 4));
fprintf('  Target place position: [%.3f, %.3f, %.3f]\n', placePosition(1:3));

if bestPlaceError > 0.02
    fprintf('WARNING: Large place error! Robot may not reach target.\n');
end

%% Pre-compute ALL Paths with Validation
fprintf('Pre-computing all paths with validation...\n');
tic;

% Task configurations
taskConfigs = {verticalConfig, pickupConfig, pickupConfig, placeConfig, placeConfig};
taskNames = {'Move to Pickup', 'Pick Object', 'Move to Place', 'Place Object'};

% Increased parameters for better accuracy
numWaypoints = 40;  % Increased for smoother path
numSteps = 10;      % Increased for better accuracy
totalFrames = 0;

allJointPaths = cell(4, 1);
allStepCounts = zeros(4, 1);

% Generate all paths with validation
for taskIdx = 1:4
    startConfig = taskConfigs{taskIdx};
    goalConfig = taskConfigs{taskIdx+1};
    
    fprintf('Task %d (%s):\n', taskIdx, taskNames{taskIdx});
    
    if norm(goalConfig - startConfig) < 0.01
        jointPath = repmat(startConfig, 1, 8);  % Longer pause for pick/place
        fprintf('  Small movement - creating pause\n');
    else
        jointPath = generateJointPathVectorized(startConfig, goalConfig, numWaypoints);
        
        % Validate final position
        T_final = getTransform(robot, jointPath(:, end), 'tool0', 'base_link');
        if taskIdx == 1
            targetPos = pickupPosition(1:3);
            targetName = 'pickup';
        elseif taskIdx == 3
            targetPos = placePosition(1:3);
            targetName = 'place';
        else
            targetPos = [];
            targetName = 'pause';
        end
        
        if ~isempty(targetPos)
            finalError = norm(T_final(1:3, 4) - targetPos);
            fprintf('  Path final error to %s: %.4f m\n', targetName, finalError);
            
            % If error is large, add extra waypoints to target
            if finalError > 0.01
                fprintf('  Adding correction waypoints...\n');
                % Add 5 more waypoints that force the robot to the exact target
                if taskIdx == 1
                    correctionTarget = pickupConfig;
                else
                    correctionTarget = placeConfig;
                end
                correctionPath = generateJointPathVectorized(jointPath(:, end), correctionTarget, 5);
                jointPath = [jointPath, correctionPath(:, 2:end)];
            end
        end
    end
    
    allJointPaths{taskIdx} = jointPath;
    allStepCounts(taskIdx) = size(jointPath, 2) * numSteps;
    totalFrames = totalFrames + allStepCounts(taskIdx);
    fprintf('  Generated %d waypoints, %d total steps\n', size(jointPath, 2), allStepCounts(taskIdx));
end

fprintf('Total frames to compute: %d\n', totalFrames);

% Pre-allocate ALL arrays for maximum speed
allConfigs = zeros(6, totalFrames);
allEEPositions = zeros(3, totalFrames);
frameToTask = zeros(1, totalFrames);
frameToWaypoint = zeros(1, totalFrames);

% Pre-compute ALL configurations and EE positions
fprintf('Pre-computing %d total frames...\n', totalFrames);
globalFrameIdx = 1;

for taskIdx = 1:4
    jointPath = allJointPaths{taskIdx};
    
    for waypointIdx = 1:size(jointPath, 2)
        if waypointIdx == 1
            if taskIdx == 1
                startConfigLocal = verticalConfig;
            else
                startConfigLocal = allConfigs(:, globalFrameIdx-1);
            end
        else
            startConfigLocal = allConfigs(:, globalFrameIdx-1);
        end
        targetConfig = jointPath(:, waypointIdx);
        
        % Generate interpolated steps
        stepPath = generateJointPathVectorized(startConfigLocal, targetConfig, numSteps);
        
        % Store all steps
        for step = 1:numSteps
            allConfigs(:, globalFrameIdx) = stepPath(:, step);
            frameToTask(globalFrameIdx) = taskIdx;
            frameToWaypoint(globalFrameIdx) = waypointIdx;
            globalFrameIdx = globalFrameIdx + 1;
        end
    end
end

% Vectorized computation of ALL end-effector positions
fprintf('Computing all EE positions (vectorized)...\n');
for i = 1:totalFrames
    T_ee = getTransform(robot, allConfigs(:, i), 'tool0', 'base_link');
    allEEPositions(:, i) = T_ee(1:3, 4);
end

precomputeTime = toc;
fprintf('Pre-computation completed in %.2f seconds\n', precomputeTime);

%% Setup Fast Visualization
imgDir = 'ur5e_fast_frames';
if exist(imgDir, 'dir'), rmdir(imgDir, 's'); end
mkdir(imgDir);

figure('Name', 'UR5e Fast Simulation', 'Position', [100, 100, 1000, 700]);
set(gcf, 'Renderer', 'opengl'); % Hardware acceleration

%% Fast Execution Loop
fprintf('Starting fast execution...\n');
tic;

objectAttached = false;
objectPosition = pickupPosition(1:3);
objectSize = [0.05, 0.05, 0.05];
objectColor = [1.0, 0.5, 0.0];

% Optimized update frequency
updateEvery = 2;  % Update visualization every 2 frames
saveEvery = 4;    % Save every 4th frame (instead of every 2nd)

savedFrameCount = 0;

for frameIdx = 1:totalFrames
    currentConfig = allConfigs(:, frameIdx);
    eePos = allEEPositions(:, frameIdx);
    currentTask = frameToTask(frameIdx);
    currentWaypoint = frameToWaypoint(frameIdx);
    
    % Object attachment logic - based on actual position with more reasonable thresholds
    distToPickup = norm(eePos - pickupPosition(1:3)');
    distToPlace = norm(eePos - placePosition(1:3)');
    
    % Pick up object when close to pickup position
    if ~objectAttached && distToPickup < 0.08 && currentTask >= 2
        objectAttached = true;
        fprintf('Frame %d: Object picked up (distance: %.4f m)\n', frameIdx, distToPickup);
    end
    
    % Place object when close to place position AND we're in the final task
    if objectAttached && currentTask == 4 && distToPlace < 0.10
        objectAttached = false;
        objectPosition = eePos;  % Place at actual gripper position
        fprintf('Frame %d: Object placed (distance: %.4f m)\n', frameIdx, distToPlace);
        fprintf('  Object placed at: [%.3f, %.3f, %.3f]\n', objectPosition);
    end
    
    % Update object position if attached
    if objectAttached
        currentObjectPos = eePos + [0; 0; 0.08];
    else
        currentObjectPos = objectPosition;
    end
    
    % Debug output for critical frames
    if currentTask == 3 && mod(frameIdx, 20) == 0  % Moving to place
        fprintf('Frame %d: Moving to place - Distance: %.4f m, EE: [%.3f,%.3f,%.3f]\n', ...
                frameIdx, distToPlace, eePos);
    end
    
    % Fast visualization update
    if mod(frameIdx, updateEvery) == 0 || frameIdx == 1 || frameIdx == totalFrames
        updateVisualizationFast(robot, currentConfig, eePos, allEEPositions(:, 1:frameIdx), ...
                              objectAttached, currentObjectPos, objectSize, objectColor, ...
                              pickupPosition, placePosition, taskNames{currentTask}, ...
                              currentWaypoint, frameIdx, currentTask);
        
        % Fast image saving
        if mod(frameIdx, saveEvery) == 0
            savedFrameCount = savedFrameCount + 1;
            filename = fullfile(imgDir, sprintf('frame_%04d.png', savedFrameCount));
            
            % Ultra-fast image capture and save
            frame = getframe(gcf);
            img = imresize(frame.cdata, 0.6);  % 60% size for speed
            imwrite(img, filename, 'png', 'Compression', 'high');
        end
        
        drawnow limitrate; % Fastest drawing
    end
    
    pause(0.015); % Faster playback
end

executionTime = toc;
totalTime = precomputeTime + executionTime;

%% Create GIF
fprintf('Creating GIF from %d saved frames...\n', savedFrameCount);
createGIFFast(imgDir, 'ur5e_fast_movement.gif');

%% Final Status and Validation
finalEEPos = allEEPositions(:, end);
finalDistToPlace = norm(finalEEPos - placePosition(1:3)');

fprintf('\n=== PERFORMANCE & STATUS ===\n');
fprintf('Total computation time: %.2f seconds\n', totalTime);
fprintf('Pre-computation: %.2f seconds\n', precomputeTime);
fprintf('Execution: %.2f seconds\n', executionTime);
fprintf('Frames processed: %d\n', totalFrames);
fprintf('Frames saved: %d\n', savedFrameCount);

fprintf('\n=== FINAL POSITIONS & VALIDATION ===\n');
fprintf('Target place position: [%.3f, %.3f, %.3f]\n', placePosition(1:3));
fprintf('Final EE position:     [%.3f, %.3f, %.3f]\n', finalEEPos);
fprintf('EE position error:     %.4f m\n', finalDistToPlace);

if finalDistToPlace < 0.10
    fprintf('✓ Robot successfully reached placement zone\n');
else
    fprintf('❌ Robot did NOT reach placement target (error: %.4f m)\n', finalDistToPlace);
    fprintf('   This indicates an IK or path planning issue.\n');
end

fprintf('Object final position: [%.3f, %.3f, %.3f]\n', objectPosition);
if ~objectAttached
    objectPlaceError = norm(objectPosition - placePosition(1:3)');
    fprintf('Object placement error: %.4f m\n', objectPlaceError);
    if objectPlaceError < 0.15
        fprintf('✓ Object successfully placed near target\n');
    else
        fprintf('❌ Object placed far from target\n');
    end
else
    fprintf('❌ Object still attached to gripper - placement failed\n');
end

fprintf('\n=== RECOMMENDATIONS ===\n');
if finalDistToPlace > 0.10
    fprintf('• Try adjusting target position to be within robot workspace\n');
    fprintf('• Check joint limits and IK solver settings\n');
    fprintf('• Consider using different initial guess for IK\n');
end

%% Cleanup
rmdir(imgDir, 's');
fprintf('Fast simulation completed!\n');

%% Optimized Helper Functions

function config = computeIK(ikSolver, pose, initialGuess)
    % Improved IK computation with better parameters
    T_target = eye(4);
    T_target(1:3, 4) = pose(1:3);
    T_target(1:3, 1:3) = eul2rotm(pose(4:6), 'XYZ');
    
    % Try with different weight combinations for better solutions
    weightSets = {
        [1, 1, 1, 0.3, 0.3, 0.3],    % Standard weights
        [1, 1, 1, 0.1, 0.1, 0.1],    % Prioritize position
        [0.8, 0.8, 0.8, 0.5, 0.5, 0.5]  % Balanced weights
    };
    
    bestConfig = initialGuess;
    bestError = inf;
    
    for i = 1:length(weightSets)
        try
            [config_temp, solInfo] = ikSolver('tool0', T_target, weightSets{i}, initialGuess);
            
            % Check solution quality
            T_result = getTransform(ikSolver.RigidBodyTree, config_temp, 'tool0', 'base_link');
            error = norm(T_result(1:3, 4) - pose(1:3)');
            
            if error < bestError && solInfo.ExitFlag > 0
                bestConfig = config_temp;
                bestError = error;
            end
            
            % If we get a very good solution, use it
            if error < 0.005
                break;
            end
        catch
            % Continue to next weight set
        end
    end
    
    config = bestConfig;
end

function jointPath = generateJointPathVectorized(startConfig, endConfig, numWaypoints)
    % Fully vectorized joint path generation
    startConfig = startConfig(:);
    endConfig = endConfig(:);
    
    % Vectorized angle wrapping
    jointDiff = endConfig - startConfig;
    jointDiff(jointDiff > pi) = jointDiff(jointDiff > pi) - 2*pi;
    jointDiff(jointDiff < -pi) = jointDiff(jointDiff < -pi) + 2*pi;
    
    % Vectorized interpolation
    t = linspace(0, 1, numWaypoints);
    jointPath = startConfig + jointDiff * t;
end

function updateVisualizationFast(robot, currentConfig, eePos, pathTrace, ...
                               objectAttached, objectPosition, objectSize, objectColor, ...
                               pickupPos, placePos, taskName, waypoint, frameNum, taskIdx)
    clf;
    
    % Minimal robot display
    show(robot, currentConfig, 'Frames', 'off', 'PreservePlot', false, 'Visuals', 'on');
    hold on;
    
    % Fast ground plane
    patch([-1 1 1 -1], [-1 -1 1 1], [0 0 0 0], [0.85, 0.85, 0.85], ...
          'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    % Object position
    plotBoxFast(objectPosition, objectSize, objectColor);
    
    % Fast path trace (sample points for speed)
    if size(pathTrace, 2) > 1
        % Sample path for faster rendering
        sampleIdx = 1:max(1, floor(size(pathTrace, 2)/100)):size(pathTrace, 2);
        plot3(pathTrace(1, sampleIdx), pathTrace(2, sampleIdx), pathTrace(3, sampleIdx), ...
              'g-', 'LineWidth', 3);
    end
    
    % Essential markers
    plot3(pickupPos(1), pickupPos(2), pickupPos(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', [0, 0.8, 0]);
    plot3(placePos(1), placePos(2), placePos(3), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', [0, 0.4, 1]);
    plot3(eePos(1), eePos(2), eePos(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
    
    % Show distance to current target
    if taskIdx <= 2
        % Moving to pickup or picking
        targetPos = pickupPos(1:3);
        targetName = 'PICKUP';
        lineColor = 'g';
    else
        % Moving to place or placing
        targetPos = placePos(1:3);
        targetName = 'PLACE';
        lineColor = 'b';
    end
    
    distToTarget = norm(eePos - targetPos);
    
    % Draw line to current target
    plot3([eePos(1), targetPos(1)], [eePos(2), targetPos(2)], [eePos(3), targetPos(3)], ...
          '--', 'Color', lineColor, 'LineWidth', 2);
    
    % Show distance text
    midPoint = (eePos + targetPos) / 2;
    text(midPoint(1), midPoint(2), midPoint(3) + 0.1, ...
         sprintf('%.3fm to %s', distToTarget, targetName), ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', 'black', ...
         'HorizontalAlignment', 'center');
    
    % Object attachment status
    if objectAttached
        text(eePos(1), eePos(2), eePos(3) + 0.15, 'ATTACHED', ...
             'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
    end
    
    % Fast lighting
    lighting gouraud;
    light('Position', [1, 1, 1], 'Style', 'infinite');
    
    % Compact title with distance info
    title(sprintf('T%d: %s | F:%d | Dist:%.3fm | EE:[%.2f,%.2f,%.2f]', ...
                  taskIdx, taskName, frameNum, distToTarget, eePos), 'FontSize', 9);
    
    % Fixed view
    axis equal; view(135, 25);
    xlim([-1, 1]); ylim([-1, 1]); zlim([0, 1.2]);
    grid on;
end

function plotBoxFast(position, size, color)
    % Ultra-fast box plotting
    x = position(1) + size(1)/2 * [-1, 1, 1, -1, -1, 1, 1, -1];
    y = position(2) + size(2)/2 * [-1, -1, 1, 1, -1, -1, 1, 1];
    z = position(3) + size(3)/2 * [-1, -1, -1, -1, 1, 1, 1, 1];
    
    % Just top face for speed
    vertices = [x; y; z]';
    faces = [5,6,7,8]; % Top face only
    
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, ...
          'FaceAlpha', 0.8, 'EdgeColor', 'none');
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
    
    % Create GIF with faster processing
    for i = 1:length(imageFiles)
        filename = fullfile(imgDir, imageFiles(i).name);
        img = imread(filename);
        [imgIndexed, cmap] = rgb2ind(img, 128); % Reduced colors for speed
        
        if i == 1
            imwrite(imgIndexed, cmap, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', 0.04);
        else
            imwrite(imgIndexed, cmap, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.04);
        end
    end
    
    fprintf('Fast GIF saved: %s\n', gifFilename);
end