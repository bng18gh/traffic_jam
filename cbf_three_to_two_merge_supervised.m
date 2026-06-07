%% cbf_three_to_two_merge_supervised.m
% Supervised CBF-QP controller for the same 3-lane-to-2-lane freeway merge.
%
% Geometry:
%   Vehicles drive in +y direction.
%   Lane 1 = leftmost / inside lane
%   Lane 2 = middle lane
%   Lane 3 = rightmost disappearing lane
%
% Control line:
%   y = 0
%   When the right-lane car crosses this line, a minimal supervisor checks
%   whether the target lane is clear in a 5 m forward / 5 m rear search window.
%
% Supervised merge behavior:
%   - If target lane is clear, the right-lane car begins merging.
%   - If target lane is blocked, the right-lane car does not merge and its
%     nominal speed becomes half the blocking vehicle's current speed.
%
% Refined CBF behavior:
%   - Collision avoidance uses a 10 m center-to-center safety distance.
%   - Collision avoidance is active only when:
%       1. either vehicle is actively merging, or
%       2. both vehicles are in the same effective lane.
%   - Vehicles in different lanes that are both not merging do not constrain
%     each other.
%
% Cohesion behavior:
%   - If a vehicle is behind another vehicle in the same effective lane,
%     nominal speed encourages it to catch up until the 10 m safety distance.
%   - At the 10 m boundary, cohesion and collision avoidance should balance.
%
% Required:
%   - MATLAB Automated Driving Toolbox
%   - Optimization Toolbox for quadprog

clear; close all; clc;

%% Parameters

params.dt = 0.1;
params.T  = 24.0;

params.laneWidth = 3.6;

% Control line is fixed at y = 0.
params.controlLineY = 0.0;

% Geometric point where lane 3 has disappeared into lane 2.
params.matchLineY = 35.0;

% CBF safety distance.
params.collisionRadius = 10.0;     % m, center-to-center minimum distance
params.cbfGamma = 1.0;

% Nominal/cohesion behavior.
params.kCohesion = 0.35;
params.vMax = 24.0;

% Acceleration limits for speed-level QP.
params.maxAccel = 3.0;     % m/s^2
params.maxDecel = 10.0;    % m/s^2

% Minimal merge-admissibility supervisor.
% Target lane must be clear in this longitudinal search window around the merging car.
params.mergeSearchAhead = 5.0;    % m ahead in target lane
params.mergeSearchBehind = 5.0;   % m behind in target lane

% Visualization.
params.pauseTime = 0.04;
params.showSafetyCircles = true;

%% Lane geometry

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
% Lane centers should align with x = -3.6 and x = 0.
downstreamCenterX = -params.laneWidth / 2;

downstreamRoadCenters = [downstreamCenterX params.matchLineY 0;
                         downstreamCenterX 140 0];

downstreamLanes = lanespec(2, 'Width', params.laneWidth);
road(scenario, downstreamRoadCenters, 'Lanes', downstreamLanes);

%% Create vehicles: same setup as baseline

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
cars(2) = makeCarState('Car_M', MIDDLE_LANE, laneX(MIDDLE_LANE), -35, 20.0, veh2);
cars(3) = makeCarState('Car_R', RIGHT_LANE,  laneX(RIGHT_LANE),  -60, 30.0, veh3);

numCars = numel(cars);

%% Metric logs

numSteps = round(params.T / params.dt) + 1;

timeLog = zeros(numSteps, 1);
globalMinDistanceLog = zeros(numSteps, 1);
activeCBFMinDistanceLog = zeros(numSteps, 1);
speedLog = zeros(numSteps, numCars);
positionLog = zeros(numSteps, numCars, 2);

%% Figure

fig = figure;
ax = axes('Parent', fig);

%% Main simulation loop

for k = 1:numSteps

    t = (k - 1) * params.dt;

    %% 1. Merge-admissibility supervisor at the control line

    for i = 1:numCars

        isRightLaneCar = cars(i).originLane == RIGHT_LANE;

        if isRightLaneCar && ...
           ~cars(i).hasStartedMerge && ...
           cars(i).y >= params.controlLineY

            [isSafeToMerge, blockingID] = isTargetLaneClearAhead( ...
                i, ...
                cars, ...
                MIDDLE_LANE, ...
                params);

            if isSafeToMerge
                cars(i).hasStartedMerge = true;
                cars(i).mergeStartY = max(cars(i).y, params.controlLineY);

                cars(i).mergeBlocked = false;
                cars(i).mergeBlockID = 0;

                fprintf('t = %.1f s: %s passed control line, safe search window found, begins CBF-regulated merge.\n', ...
                    t, cars(i).name);

            else
                cars(i).hasStartedMerge = false;
                cars(i).mergeStartY = inf;

                cars(i).mergeBlocked = true;
                cars(i).mergeBlockID = blockingID;

                fprintf('t = %.1f s: %s passed control line, merge blocked by %s, slowing to half speed.\n', ...
                    t, cars(i).name, cars(blockingID).name);
            end
        end
    end

    %% 2. Solve centralized CBF-QP for safe y-speeds

    [safeSpeeds, nominalSpeeds, qpExitflag] = solveCBFSpeeds(cars, laneX, params);

    %% 3. Update vehicle states

    for i = 1:numCars

        oldX = cars(i).x;
        oldY = cars(i).y;

        cars(i).speed = safeSpeeds(i);

        % Here speed means y-direction progress rate.
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

    %% 4. Log metrics

    timeLog(k) = t;
    globalMinDistanceLog(k) = computeMinimumDistanceAll(cars);
    activeCBFMinDistanceLog(k) = computeMinimumDistanceActiveCBFPairs(cars, params);

    for i = 1:numCars
        speedLog(k,i) = cars(i).speed;
        positionLog(k,i,:) = [cars(i).x, cars(i).y];
    end

    %% 5. Draw animation in one figure

    cla(ax);
    plot(scenario, 'Parent', ax);
    hold(ax, 'on');

    % Control line.
    drawHorizontalLine(ax, laneX, params, params.controlLineY, 'r-', 2.0);
    text(ax, laneX(RIGHT_LANE)+1.0, params.controlLineY, 1.2, ...
        'control line', 'Color', 'r', 'FontSize', 8);

    % Merge guide: right lane into middle lane.
    plot3(ax, ...
        [laneX(RIGHT_LANE), laneX(MIDDLE_LANE)], ...
        [params.controlLineY, params.matchLineY], ...
        [0.3, 0.3], ...
        'm--', ...
        'LineWidth', 2);

    % Safety circles.
    if params.showSafetyCircles
        for i = 1:numCars
            drawCircle2D(ax, [cars(i).x, cars(i).y], params.collisionRadius);
        end
    end

    % Vehicle labels.
    for i = 1:numCars
        effectiveLane = getEffectiveLane(cars(i), params);

        if isVehicleMerging(cars(i), params)
            mergeText = 'MERGE';
        elseif cars(i).mergeBlocked
            mergeText = 'BLOCKED';
        else
            mergeText = 'NO-MERGE';
        end

        text(ax, ...
            cars(i).x + 0.6, ...
            cars(i).y, ...
            1.0, ...
            sprintf('%s | v=%.1f | vnom=%.1f | lane=%d | %s', ...
                cars(i).name, cars(i).speed, nominalSpeeds(i), effectiveLane, mergeText), ...
            'FontSize', 8);
    end

    if qpExitflag > 0
        qpStatus = 'QP solved';
    else
        qpStatus = sprintf('QP fallback, flag=%d', qpExitflag);
    end

    if isinf(activeCBFMinDistanceLog(k))
        activeDistText = 'none';
    else
        activeDistText = sprintf('%.1f m', activeCBFMinDistanceLog(k));
    end

    title(ax, sprintf('Supervised CBF-QP 3-to-2 Merge | t = %.1f s | active CBF min dist = %s | %s', ...
        t, activeDistText, qpStatus));

    xlabel(ax, 'x position [m]');
    ylabel(ax, 'y position [m]');

    xlim(ax, [-15, 12]);
    ylim(ax, [-80, 95]);

    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 2);

    hold(ax, 'off');

    drawnow limitrate;
    pause(params.pauseTime);
end

%% Plot metrics after animation

figure;
plot(timeLog, globalMinDistanceLog, 'LineWidth', 1.5);
hold on;
plot(timeLog, activeCBFMinDistanceLog, 'LineWidth', 1.5);
yline(params.collisionRadius, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Distance [m]');
title('Supervised CBF Safety Metrics');
legend('Global minimum distance', 'Active-CBF-pair minimum distance', 'Collision radius', ...
    'Location', 'best');


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

    car.yaw = 90;

    car.hasStartedMerge = originLane ~= 3;
    car.mergeStartY = inf;

    % Merge-admissibility supervisor fields.
    car.mergeBlocked = false;
    car.mergeBlockID = 0;

    car.obj = vehicleObj;
end


function [safeSpeeds, nominalSpeeds, exitflag] = solveCBFSpeeds(cars, laneX, params)
%SOLVECBFSPEEDS Centralized speed-level CBF-QP.
%
% Decision variable:
%   v = [v_1; v_2; ...; v_N]
%
% Objective:
%   minimize sum_i (v_i - v_nom_i)^2
%
% Constraints:
%   pairwise CBF collision avoidance
%   speed bounds
%   acceleration bounds

    numCars = numel(cars);

    positions = zeros(numCars, 2);
    pathDerivatives = zeros(numCars, 2);

    for i = 1:numCars
        positions(i,:) = [cars(i).x, cars(i).y];
        pathDerivatives(i,:) = getPathDerivative(cars(i), laneX, params);
    end

    nominalSpeeds = computeNominalSpeeds(cars, params);

    H = eye(numCars);
    f = -nominalSpeeds(:);

    A = [];
    b = [];

    % Pairwise collision-avoidance CBF constraints.
    % Collision avoidance is only enforced when:
    %   1. car i is merging, or
    %   2. car j is merging, or
    %   3. both cars are in the same effective lane.
    for i = 1:numCars-1
        for j = i+1:numCars

            applyCBF = shouldApplyCollisionCBF(i, j, cars, params);

            if ~applyCBF
                continue;
            end

            p_i = positions(i,:).';
            p_j = positions(j,:).';

            q_i = pathDerivatives(i,:).';
            q_j = pathDerivatives(j,:).';

            r_ij = p_i - p_j;

            h_ij = dot(r_ij, r_ij) - params.collisionRadius^2;

            % h_dot = 2*r'*(q_i*v_i - q_j*v_j)
            % CBF: h_dot + gamma*h >= 0
            % A*v <= b:
            %   -2*r'*q_i*v_i + 2*r'*q_j*v_j <= gamma*h

            Arow = zeros(1, numCars);

            Arow(i) = -2 * (r_ij.' * q_i);
            Arow(j) =  2 * (r_ij.' * q_j);

            brow = params.cbfGamma * h_ij;

            A = [A; Arow]; %#ok<AGROW>
            b = [b; brow]; %#ok<AGROW>
        end
    end

    % Speed and acceleration bounds.
    lb = zeros(numCars, 1);
    ub = params.vMax * ones(numCars, 1);

    for i = 1:numCars
        lb(i) = max(0, cars(i).speed - params.maxDecel * params.dt);
        ub(i) = min(params.vMax, cars(i).speed + params.maxAccel * params.dt);
    end

    options = optimset('Display', 'off');

    [v, ~, exitflag] = quadprog( ...
        H, f, ...
        A, b, ...
        [], [], ...
        lb, ub, ...
        [], ...
        options);

    % Safety-first fallback:
    % if acceleration bounds make the QP infeasible, retry using only
    % physical speed bounds.
    if exitflag <= 0

        lbFallback = zeros(numCars, 1);
        ubFallback = params.vMax * ones(numCars, 1);

        [v, ~, exitflag] = quadprog( ...
            H, f, ...
            A, b, ...
            [], [], ...
            lbFallback, ubFallback, ...
            [], ...
            options);
    end

    if exitflag > 0
        safeSpeeds = v(:).';
    else
        warning('CBF-QP infeasible even after fallback. Commanding zero speeds.');
        safeSpeeds = zeros(1, numCars);
    end
end


function nominalSpeeds = computeNominalSpeeds(cars, params)
%COMPUTENOMINALSPEEDS Cruise + same-lane cohesion behavior.
%
% If a right-lane vehicle has crossed the control line but merge is blocked:
%   v_nom = 0.5 * speed(blocking vehicle)
%
% Otherwise:
%   If no lead car exists in the same effective lane:
%       v_nom = cruise speed
%
%   If a lead car exists:
%       v_nom = leadSpeed + kCohesion * (gap - collisionRadius)
%
% At gap = collisionRadius:
%   v_nom = leadSpeed

    numCars = numel(cars);
    nominalSpeeds = zeros(numCars, 1);

    for i = 1:numCars

        % Minimal supervisor behavior:
        % If the right-lane car is blocked from merging, slow to half the
        % speed of the blocking car.
        if cars(i).mergeBlocked && cars(i).mergeBlockID ~= 0
            blockID = cars(i).mergeBlockID;

            nominalSpeeds(i) = 0.5 * cars(blockID).speed;
            nominalSpeeds(i) = min(max(nominalSpeeds(i), 0), params.vMax);

            continue;
        end

        effectiveLane = getEffectiveLane(cars(i), params);

        leadID = findLeadVehicleInEffectiveLane(i, cars, effectiveLane, params);

        if leadID == 0
            nominalSpeeds(i) = cars(i).cruiseSpeed;
        else
            gap = cars(leadID).y - cars(i).y;

            nominalSpeeds(i) = cars(leadID).speed + ...
                params.kCohesion * (gap - params.collisionRadius);
        end

        nominalSpeeds(i) = min(max(nominalSpeeds(i), 0), params.vMax);
    end
end


function [isClear, blockingID] = isTargetLaneClearAhead(i, cars, targetLane, params)
%ISTARGETLANECLEARAHEAD Check whether target lane is clear around the merge point.
%
% Unsafe if there is a vehicle in the target lane within the longitudinal window:
%
%   -mergeSearchBehind <= y_j - y_i <= mergeSearchAhead
%
% This catches vehicles directly beside the merging car, slightly ahead, and
% slightly behind. The function name is kept unchanged to avoid modifying the
% caller, but the logic is now a forward/backward search window.

    egoY = cars(i).y;

    isClear = true;
    blockingID = 0;
    closestAbsGap = inf;

    for j = 1:numel(cars)

        if j == i
            continue;
        end

        otherLane = getEffectiveLane(cars(j), params);

        if otherLane ~= targetLane
            continue;
        end

        longitudinalGap = cars(j).y - egoY;

        isWithinSearchWindow = longitudinalGap >= -params.mergeSearchBehind && ...
                               longitudinalGap <=  params.mergeSearchAhead;

        if isWithinSearchWindow
            isClear = false;

            % Track the closest blocker in the search window for diagnostics
            % and for the nominal half-speed behavior.
            if abs(longitudinalGap) < closestAbsGap
                closestAbsGap = abs(longitudinalGap);
                blockingID = j;
            end
        end
    end
end


function leadID = findLeadVehicleInEffectiveLane(i, cars, laneID, params)
%FINDLEADVEHICLEINEFFECTIVELANE Find nearest vehicle ahead in same lane.

    leadID = 0;
    bestGap = inf;

    for j = 1:numel(cars)

        if j == i
            continue;
        end

        otherLane = getEffectiveLane(cars(j), params);

        if otherLane ~= laneID
            continue;
        end

        gap = cars(j).y - cars(i).y;

        if gap > 0 && gap < bestGap
            bestGap = gap;
            leadID = j;
        end
    end
end


function applyCBF = shouldApplyCollisionCBF(i, j, cars, params)
%SHOULDAPPLYCOLLISIONCBF Decide whether pairwise CBF should be active.
%
% Collision avoidance is active only if:
%   - either vehicle is currently merging, or
%   - both vehicles are in the same effective lane.

    iIsMerging = isVehicleMerging(cars(i), params);
    jIsMerging = isVehicleMerging(cars(j), params);

    sameEffectiveLane = getEffectiveLane(cars(i), params) == ...
                        getEffectiveLane(cars(j), params);

    applyCBF = iIsMerging || jIsMerging || sameEffectiveLane;
end


function isMerging = isVehicleMerging(car, params)
%ISVEHICLEMERGING True if car is actively changing from lane 3 to lane 2.

    RIGHT_LANE = 3;

    isMerging = car.originLane == RIGHT_LANE && ...
                car.hasStartedMerge && ...
                car.y >= params.controlLineY && ...
                car.y < params.matchLineY;
end


function effectiveLane = getEffectiveLane(car, params)
%GETEFFECTIVELANE Lane used for cohesion/following logic.
%
% A right-lane vehicle is considered part of the middle lane once it starts
% the merge after crossing the control line.

    RIGHT_LANE = 3;
    MIDDLE_LANE = 2;

    if car.originLane == RIGHT_LANE && car.hasStartedMerge
        effectiveLane = MIDDLE_LANE;
    else
        effectiveLane = car.currentLane;
    end
end


function q = getPathDerivative(car, laneX, params)
%GETPATHDERIVATIVE Return p_dot per unit y-speed.
%
% If v_i is the y-speed decision variable:
%   p_dot_i = q_i * v_i
%
% For straight lanes:
%   q_i = [0, 1]
%
% For the merging car:
%   q_i = [dx/dy, 1]

    RIGHT_LANE = 3;
    MIDDLE_LANE = 2;

    if car.originLane == RIGHT_LANE && car.hasStartedMerge && car.y < params.matchLineY
        dxdy = (laneX(MIDDLE_LANE) - laneX(RIGHT_LANE)) / ...
               (params.matchLineY - params.controlLineY);

        q = [dxdy, 1];
    else
        q = [0, 1];
    end
end


function [x, currentLane] = computeLanePosition(car, laneX, params)
%COMPUTELANEPOSITION Compute x position from current y.
%
% Right-lane car stays in lane 3 until the supervisor allows merging.
% After merge activation, it moves from lane 3 to lane 2.
% After match line, it is in lane 2.

    RIGHT_LANE = 3;
    MIDDLE_LANE = 2;

    if car.originLane ~= RIGHT_LANE
        x = laneX(car.originLane);
        currentLane = car.originLane;
        return;
    end

    if ~car.hasStartedMerge || car.y <= params.controlLineY
        x = laneX(RIGHT_LANE);
        currentLane = RIGHT_LANE;
        return;
    end

    if car.y < params.matchLineY

        s = (car.y - params.controlLineY) / ...
            (params.matchLineY - params.controlLineY);

        s = min(max(s, 0), 1);

        x = (1 - s) * laneX(RIGHT_LANE) + s * laneX(MIDDLE_LANE);
        currentLane = RIGHT_LANE;
        return;
    end

    x = laneX(MIDDLE_LANE);
    currentLane = MIDDLE_LANE;
end


function dmin = computeMinimumDistanceAll(cars)

    dmin = inf;

    for i = 1:numel(cars)-1
        for j = i+1:numel(cars)
            pi = [cars(i).x, cars(i).y];
            pj = [cars(j).x, cars(j).y];

            d = norm(pi - pj);

            if d < dmin
                dmin = d;
            end
        end
    end
end


function dmin = computeMinimumDistanceActiveCBFPairs(cars, params)

    dmin = inf;

    for i = 1:numel(cars)-1
        for j = i+1:numel(cars)

            if ~shouldApplyCollisionCBF(i, j, cars, params)
                continue;
            end

            pi = [cars(i).x, cars(i).y];
            pj = [cars(j).x, cars(j).y];

            d = norm(pi - pj);

            if d < dmin
                dmin = d;
            end
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


function drawCircle2D(ax, center, radius)

    theta = linspace(0, 2*pi, 100);

    x = center(1) + radius*cos(theta);
    y = center(2) + radius*sin(theta);

    plot3(ax, x, y, 0.25*ones(size(x)), ...
        'k:', ...
        'LineWidth', 0.75);
end
