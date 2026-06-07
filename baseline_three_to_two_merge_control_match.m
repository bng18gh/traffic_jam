%% baseline_three_to_two_merge_control_match.m
% Rule-based baseline controller for a 3-lane-to-2-lane freeway merge.
%
% Geometry:
%   Vehicles drive in +y direction.
%   Lane 1 = leftmost / inside lane
%   Lane 2 = middle lane
%   Lane 3 = rightmost disappearing lane
%
% Lines:
%   Control line: y = 0
%       - right-of-way / yield decision happens here
%       - yielding right-lane car scans the lane next over before merging
%
%   Match line: y = matchLineY
%       - lane 3 has disappeared into lane 2
%       - yielded / merged car matches current speed of vehicle in front
%
% Baseline behavior:
%   - ROW vehicle continues at cruise speed.
%   - Yielding right-lane vehicle scans the target lane ahead.
%   - If unsafe, it stays in the right lane and rapidly decelerates to 2 m/s.
%   - If safe, it begins merging while decelerating to ROW speed - 2 m/s.
%   - After the match line, the merged vehicle matches the current speed of
%     the nearest vehicle ahead in its lane.

clear; close all; clc;

%% Parameters

params.dt = 0.1;
params.T  = 24.0;

params.laneWidth = 3.6;

% Control line is fixed at y = 0.
params.controlLineY = 0.0;

% Match line is where lane 3 disappears into lane 2.
params.matchLineY = 35.0;

% Speed and following parameters.
params.desiredGap = 14.0;
params.emergencyGap = 8.0;
params.followDetectRange = 100.0;

params.maxAccel = 2.5;      % m/s^2
params.maxDecel = 8.0;      % m/s^2, rapid yielding deceleration

% Faster acceleration after merge / match.
params.matchAccel = 6.0;    % m/s^2

% Yielding car should be at least this much slower than ROW car before match line.
params.yieldSpeedMargin = 2.0;  % m/s

% Merge safety check.
params.mergeSafetyHorizon = 10.0;   % meters ahead in target lane
params.unsafeYieldSpeed = 5.0;     % m/s when unsafe to merge

% Visualization.
params.pauseTime = 0.04;

%% Lane geometry

% Vehicles move in +y direction.
% Facing +y:
%   leftmost lane  = negative x
%   rightmost lane = positive x

laneX = [-params.laneWidth, 0, params.laneWidth];

LEFT_LANE   = 1;
MIDDLE_LANE = 2;
RIGHT_LANE  = 3;

%% Create driving scenario

scenario = drivingScenario( ...
    'SampleTime', params.dt, ...
    'StopTime', params.T);

% Upstream 3-lane road from y = -120 to match line.
upstreamRoadCenters = [0 -120 0;
                       0 params.matchLineY 0];

upstreamLanes = lanespec(3, 'Width', params.laneWidth);
road(scenario, upstreamRoadCenters, 'Lanes', upstreamLanes);

% Downstream 2-lane road.
% We want downstream lane centers to align with x = -3.6 and x = 0.
% Therefore the 2-lane road centerline is at x = -1.8.
downstreamCenterX = -params.laneWidth / 2;

downstreamRoadCenters = [downstreamCenterX params.matchLineY 0;
                         downstreamCenterX 140 0];

downstreamLanes = lanespec(2, 'Width', params.laneWidth);
road(scenario, downstreamRoadCenters, 'Lanes', downstreamLanes);

%% Create vehicles

% These initial positions/speeds are chosen so that the middle-lane and
% right-lane vehicles interact near the control/match region.

veh1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Name', 'Car_L', ...
    'Position', [laneX(LEFT_LANE) -5 0], ...
    'Yaw', 90);

veh2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Name', 'Car_M', ...
    'Position', [laneX(MIDDLE_LANE) -35 0], ...
    'Yaw', 90);

veh3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Name', 'Car_R', ...
    'Position', [laneX(RIGHT_LANE) -60 0], ...
    'Yaw', 90);

cars(1) = makeCarState('Car_L', LEFT_LANE,   laneX(LEFT_LANE),   -5,  12.0, veh1);
cars(2) = makeCarState('Car_M', MIDDLE_LANE, laneX(MIDDLE_LANE), -35, 14.0, veh2);
cars(3) = makeCarState('Car_R', RIGHT_LANE,  laneX(RIGHT_LANE),  -60, 18.0, veh3);

numCars = numel(cars);

%% Controller state

activeROW = 0;   % current vehicle with right of way through merge region

%% Figure

fig = figure;
ax = axes('Parent', fig);

%% Main simulation loop

numSteps = round(params.T / params.dt) + 1;

for k = 1:numSteps

    t = (k - 1) * params.dt;

    %% 1. Detect vehicles crossing the control line

    for i = 1:numCars

        if ~cars(i).arrivedAtControlLine

            yNow = cars(i).y;
            yNext = cars(i).y + cars(i).speed * params.dt;

            if yNow >= params.controlLineY || yNext >= params.controlLineY

                cars(i).arrivedAtControlLine = true;

                if cars(i).speed > 1e-6
                    frac = (params.controlLineY - yNow) / ...
                           max(cars(i).speed * params.dt, 1e-6);
                    frac = min(max(frac, 0), 1);
                    cars(i).arrivalTime = t + frac * params.dt;
                else
                    cars(i).arrivalTime = t;
                end

                fprintf('t = %.1f s: %s reached control line from lane %d.\n', ...
                    cars(i).arrivalTime, cars(i).name, cars(i).originLane);
            end
        end
    end

    %% 2. Assign right-of-way / yielding behavior at control line

    candidates = find([cars.arrivedAtControlLine] & ~[cars.hasDecision]);

    if ~isempty(candidates)

        if activeROW == 0
            % No current ROW vehicle. Choose one from candidates.
            winnerID = chooseRightOfWay(candidates, cars);

            cars(winnerID).mode = 'ROW';
            cars(winnerID).hasDecision = true;

            % If right-lane car has ROW, it may merge immediately because it
            % is not yielding to anyone.
            if cars(winnerID).originLane == RIGHT_LANE
                cars(winnerID).mergeAllowed = true;
                cars(winnerID).hasStartedMerge = true;
                cars(winnerID).mergeStartY = max(cars(winnerID).y, params.controlLineY);
            end

            activeROW = winnerID;

            fprintf('t = %.1f s: %s given right of way.\n', ...
                t, cars(winnerID).name);

            % Everyone else at the control line yields to the winner.
            for idx = 1:numel(candidates)
                id = candidates(idx);

                if id ~= winnerID
                    cars(id).mode = 'YIELD';
                    cars(id).hasDecision = true;
                    cars(id).yieldToID = winnerID;

                    fprintf('t = %.1f s: %s yields to %s.\n', ...
                        t, cars(id).name, cars(winnerID).name);
                end
            end

        else
            % A ROW vehicle is already in the merge region.
            % Newly arriving vehicles yield to it.
            for idx = 1:numel(candidates)
                id = candidates(idx);

                cars(id).mode = 'YIELD';
                cars(id).hasDecision = true;
                cars(id).yieldToID = activeROW;

                fprintf('t = %.1f s: %s yields to active ROW vehicle %s.\n', ...
                    t, cars(id).name, cars(activeROW).name);
            end
        end
    end

    %% 3. If ROW vehicle has passed match line and created gap, clear ROW

    if activeROW ~= 0
        if cars(activeROW).y >= params.matchLineY + params.desiredGap
            fprintf('t = %.1f s: %s cleared merge decision region.\n', ...
                t, cars(activeROW).name);

            activeROW = 0;
        end
    end

    %% 4. Check whether yielding right-lane vehicles are allowed to merge

    for i = 1:numCars

        isRightLaneYieldingCar = strcmp(cars(i).mode, 'YIELD') && ...
                                 cars(i).originLane == RIGHT_LANE;

        if isRightLaneYieldingCar && ~cars(i).hasStartedMerge

            [isSafeToMerge, blockingID] = isTargetLaneClearAhead( ...
                i, ...
                cars, ...
                MIDDLE_LANE, ...
                params);

            if isSafeToMerge
                cars(i).mergeAllowed = true;
                cars(i).hasStartedMerge = true;
                cars(i).mergeStartY = max(cars(i).y, params.controlLineY);

                fprintf('t = %.1f s: %s scanned target lane, found safe gap, begins merging.\n', ...
                    t, cars(i).name);
            else
                cars(i).mergeAllowed = false;
                cars(i).hasStartedMerge = false;
                cars(i).mergeStartY = inf;

                fprintf('t = %.1f s: %s scanned target lane, blocked by %s, holds lane and slows.\n', ...
                    t, cars(i).name, cars(blockingID).name);
            end
        end
    end

    %% 5. Compute commanded speeds

    for i = 1:numCars

        switch cars(i).mode

            case 'CRUISE'
                cars(i).cmdSpeed = cars(i).cruiseSpeed;

            case 'ROW'
                % Right-of-way vehicle does not stop.
                cars(i).cmdSpeed = cars(i).cruiseSpeed;

            case 'YIELD'
                % Yielding car either waits for safe merge gap or merges slowly.
                cars(i).cmdSpeed = computeYieldSpeed(i, cars, params);

            case 'MATCH'
                % After match line, match current speed of the car in front.
                cars(i).cmdSpeed = computeMatchSpeed(i, cars, params);

            otherwise
                cars(i).cmdSpeed = cars(i).cruiseSpeed;
        end
    end

    %% 6. Transition yielded cars to MATCH mode at match line

    for i = 1:numCars
        if strcmp(cars(i).mode, 'YIELD') && ...
           cars(i).y >= params.matchLineY && ...
           cars(i).hasStartedMerge

            cars(i).mode = 'MATCH';

            fprintf('t = %.1f s: %s reached match line and begins speed matching.\n', ...
                t, cars(i).name);
        end
    end

    %% 7. Smooth speeds, integrate motion, update MATLAB vehicle objects

    for i = 1:numCars

        oldX = cars(i).x;
        oldY = cars(i).y;

        cars(i).speed = applySpeedCommand( ...
            cars(i).speed, ...
            cars(i).cmdSpeed, ...
            params, ...
            cars(i).mode);

        cars(i).y = cars(i).y + cars(i).speed * params.dt;

        [cars(i).x, cars(i).currentLane] = computeLanePosition(cars(i), laneX, params);

        dx = cars(i).x - oldX;
        dy = cars(i).y - oldY;

        if hypot(dx, dy) > 1e-8
            cars(i).yaw = atan2d(dy, dx);
        end

        cars(i).obj.Position = [cars(i).x, cars(i).y, 0];
        cars(i).obj.Velocity = [dx / params.dt, dy / params.dt, 0];
        cars(i).obj.Yaw = cars(i).yaw;
    end

    %% 8. Draw animation in one figure

    cla(ax);
    plot(scenario, 'Parent', ax);
    hold(ax, 'on');

    % Control line.
    drawHorizontalLine(ax, laneX, params, params.controlLineY, 'r-', 2.0);
    text(ax, laneX(RIGHT_LANE)+1.0, params.controlLineY, 1.2, ...
        'control line', 'Color', 'r', 'FontSize', 8);

    % Match line.
    drawHorizontalLine(ax, laneX, params, params.matchLineY, 'k-', 2.0);
    text(ax, laneX(RIGHT_LANE)+1.0, params.matchLineY, 1.2, ...
        'match line', 'Color', 'k', 'FontSize', 8);

    % Merge guide: right lane into middle lane between control and match line.
    plot3(ax, ...
        [laneX(RIGHT_LANE), laneX(MIDDLE_LANE)], ...
        [params.controlLineY, params.matchLineY], ...
        [0.3, 0.3], ...
        'm--', ...
        'LineWidth', 2);

    % Vehicle labels.
    for i = 1:numCars
        text(ax, ...
            cars(i).x + 0.6, ...
            cars(i).y, ...
            1.0, ...
            sprintf('%s | %.1f m/s | %s', cars(i).name, cars(i).speed, cars(i).mode), ...
            'FontSize', 8);
    end

    if activeROW == 0
        activeText = 'none';
    else
        activeText = cars(activeROW).name;
    end

    title(ax, sprintf('3-to-2 Merge Baseline | t = %.1f s | ROW = %s', ...
        t, activeText));

    xlabel(ax, 'x position [m]');
    ylabel(ax, 'y position [m]');

    xlim(ax, [-12, 8]);
    ylim(ax, [-80, 95]);

    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 2);

    hold(ax, 'off');

    drawnow limitrate;
    pause(params.pauseTime);
end


%% ------------------------------------------------------------------------
% Local functions
% -------------------------------------------------------------------------

function car = makeCarState(name, originLane, x0, y0, cruiseSpeed, vehicleObj)

    car.name = name;

    car.originLane = originLane;
    car.currentLane = originLane;

    car.x = x0;
    car.y = y0;

    car.speed = cruiseSpeed;
    car.cruiseSpeed = cruiseSpeed;
    car.cmdSpeed = cruiseSpeed;

    car.yaw = 90;

    car.arrivedAtControlLine = false;
    car.arrivalTime = inf;

    car.hasDecision = false;
    car.yieldToID = 0;

    car.mode = 'CRUISE';

    car.mergeAllowed = originLane ~= 3;
    car.hasStartedMerge = originLane ~= 3;
    car.mergeStartY = inf;

    car.obj = vehicleObj;
end


function winnerID = chooseRightOfWay(candidateIDs, cars)
%CHOOSERIGHTOFWAY Apply baseline right-of-way rule.
%
% If one vehicle:
%   it goes.
%
% If two vehicles:
%   - if lanes are not adjacent, farthest-left lane wins.
%   - if lanes are adjacent, earliest arrival wins.
%   - if tied, farthest-left lane wins.
%
% If more than two:
%   deterministic extension:
%   earliest arrival wins; tie goes to farthest-left lane.

    if numel(candidateIDs) == 1
        winnerID = candidateIDs(1);
        return;
    end

    if numel(candidateIDs) == 2

        id1 = candidateIDs(1);
        id2 = candidateIDs(2);

        lane1 = cars(id1).originLane;
        lane2 = cars(id2).originLane;

        areAdjacent = abs(lane1 - lane2) == 1;

        if ~areAdjacent
            if lane1 < lane2
                winnerID = id1;
            else
                winnerID = id2;
            end
            return;
        end

        t1 = cars(id1).arrivalTime;
        t2 = cars(id2).arrivalTime;

        if t1 < t2
            winnerID = id1;
        elseif t2 < t1
            winnerID = id2;
        else
            if lane1 < lane2
                winnerID = id1;
            else
                winnerID = id2;
            end
        end

        return;
    end

    arrivalTimes = [cars(candidateIDs).arrivalTime];
    lanes = [cars(candidateIDs).originLane];

    tableToSort = [arrivalTimes(:), lanes(:), candidateIDs(:)];

    sorted = sortrows(tableToSort, [1, 2]);

    winnerID = sorted(1, 3);
end


function speedCommand = computeYieldSpeed(i, cars, params)
%COMPUTEYIELDSPEED Yield before match line.
%
% Behavior:
%   If yielding car has not been cleared to merge:
%       stay in right lane and rapidly decelerate to 2 m/s.
%
%   If yielding car is cleared to merge:
%       merge while driving 2 m/s slower than the ROW vehicle.
%
%   If too close to ROW vehicle:
%       slow even more.

    yieldToID = cars(i).yieldToID;

    if yieldToID == 0
        speedCommand = cars(i).cruiseSpeed;
        return;
    end

    rowSpeed = cars(yieldToID).speed;
    rowY = cars(yieldToID).y;

    egoY = cars(i).y;
    gap = rowY - egoY;

    isRightLaneYieldingCar = cars(i).originLane == 3;

    % Case 1:
    % Right-lane yielding car is NOT cleared to merge yet.
    % It must hold its lane and slow to 2 m/s.
    if isRightLaneYieldingCar && ~cars(i).hasStartedMerge

        speedCommand = params.unsafeYieldSpeed;

        % Do not let it drive beyond the match line while still not merged.
        distanceToMatchLine = params.matchLineY - egoY;

        if distanceToMatchLine <= 0
            speedCommand = 0;
        else
            maxSpeedBeforeMatch = distanceToMatchLine / params.dt;
            speedCommand = min(speedCommand, maxSpeedBeforeMatch);
        end

        return;
    end

    % Case 2:
    % Merge is allowed. The yielding car merges while decelerating to
    % 2 m/s slower than the ROW vehicle.
    targetYieldSpeed = max(rowSpeed - params.yieldSpeedMargin, 0);

    speedCommand = min(cars(i).cruiseSpeed, targetYieldSpeed);

    % If too close to ROW vehicle, slow more aggressively.
    if gap < params.emergencyGap
        speedCommand = max(rowSpeed - 2*params.yieldSpeedMargin, 0);
    end

    % If overlapping/ahead relative to ROW car, brake hard.
    if gap <= 0
        speedCommand = 0;
    end
end


function speedCommand = computeMatchSpeed(i, cars, params)
%COMPUTEMATCHSPEED Match current speed of nearest vehicle ahead in same lane.

    leadID = findLeadVehicleInSameLane(i, cars, cars(i).currentLane, params);

    if leadID == 0
        speedCommand = cars(i).cruiseSpeed;
        return;
    end

    gap = cars(leadID).y - cars(i).y;

    % Match the current speed of the lead vehicle.
    speedCommand = cars(leadID).speed;

    % If too close, stay below the lead vehicle's current speed.
    if gap < params.desiredGap
        speedCommand = max(cars(leadID).speed - params.yieldSpeedMargin, 0);
    end
end


function newSpeed = applySpeedCommand(currentSpeed, commandSpeed, params, mode)

    commandSpeed = max(commandSpeed, 0);

    if strcmp(mode, 'MATCH')
        maxAccel = params.matchAccel;
    else
        maxAccel = params.maxAccel;
    end

    if commandSpeed > currentSpeed
        newSpeed = min(commandSpeed, currentSpeed + maxAccel * params.dt);
    else
        newSpeed = max(commandSpeed, currentSpeed - params.maxDecel * params.dt);
    end
end


function [x, currentLane] = computeLanePosition(car, laneX, params)
%COMPUTELANEPOSITION Compute x position.
%
% Right-lane car stays in lane 3 until the merge scan passes.
% Only after hasStartedMerge becomes true does it interpolate into lane 2.

    RIGHT_LANE = 3;
    MIDDLE_LANE = 2;

    if car.originLane ~= RIGHT_LANE
        x = laneX(car.originLane);
        currentLane = car.originLane;
        return;
    end

    % Before control line, stay in right lane.
    if car.y <= params.controlLineY
        x = laneX(RIGHT_LANE);
        currentLane = RIGHT_LANE;
        return;
    end

    % If not cleared to merge, keep it in the disappearing lane.
    if ~car.hasStartedMerge
        x = laneX(RIGHT_LANE);
        currentLane = RIGHT_LANE;
        return;
    end

    % If merge has started, interpolate from lane 3 to lane 2.
    if car.y < params.matchLineY

        mergeStartY = car.mergeStartY;

        if ~isfinite(mergeStartY)
            mergeStartY = params.controlLineY;
        end

        denom = params.matchLineY - mergeStartY;

        if denom <= 1e-6
            s = 1;
        else
            s = (car.y - mergeStartY) / denom;
        end

        s = min(max(s, 0), 1);

        x = (1 - s) * laneX(RIGHT_LANE) + s * laneX(MIDDLE_LANE);
        currentLane = RIGHT_LANE;
        return;
    end

    % Past match line, right-lane vehicle is now in middle lane.
    x = laneX(MIDDLE_LANE);
    currentLane = MIDDLE_LANE;
end


function [isClear, blockingID] = isTargetLaneClearAhead(i, cars, targetLane, params)
%ISTARGETLANECLEARAHEAD Check if target lane is clear within forward horizon.
%
% Unsafe if there is a vehicle in the target lane within:
%
%   0 <= y_j - y_i <= mergeSafetyHorizon

    egoY = cars(i).y;

    isClear = true;
    blockingID = 0;

    for j = 1:numel(cars)

        if j == i
            continue;
        end

        if cars(j).currentLane ~= targetLane
            continue;
        end

        forwardGap = cars(j).y - egoY;

        if forwardGap >= 0 && forwardGap <= params.mergeSafetyHorizon
            isClear = false;
            blockingID = j;
            return;
        end
    end
end


function leadID = findLeadVehicleInSameLane(i, cars, laneID, params)

    leadID = 0;
    bestGap = inf;

    for j = 1:numel(cars)

        if j == i
            continue;
        end

        if cars(j).currentLane ~= laneID
            continue;
        end

        gap = cars(j).y - cars(i).y;

        if gap > 0 && gap < bestGap && gap < params.followDetectRange
            bestGap = gap;
            leadID = j;
        end
    end
end


function drawHorizontalLine(ax, laneX, params, yValue, lineStyle, lineWidth)

    leftEdge = laneX(1) - params.laneWidth/2;
    rightEdge = laneX(3) + params.laneWidth/2;

    plot3(ax, ...
        [leftEdge, rightEdge], ...
        [yValue, yValue], ...
        [0.2, 0.2], ...
        lineStyle, ...
        'LineWidth', lineWidth);
end
