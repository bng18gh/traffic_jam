%% run_merge_controller_metrics.m
% Metric runner for the 3-lane-to-2-lane freeway merge project.
%
% This script runs two test scenarios for two controllers:
%   1. rule-based baseline controller
%   2. supervised CBF-QP controller
%
% Test scenarios:
%   A. two cars in the middle lane, one car in the merge lane
%   B. two cars in the merge lane, one car in the middle lane
%
% Logged metrics:
%   - completion time
%   - collision count
%   - near-miss count
%   - throughput
%   - QP infeasibility count
%   - mean/max CBF intervention magnitude
%   - mean/last merge completion time
%
% Required for CBF controller:
%   - Optimization Toolbox, for quadprog

clear; close all; clc;

%% Parameters

params.dt = 0.1;
params.T  = 30.0;

params.laneWidth = 3.6;

% Road geometry.
params.controlLineY = 0.0;
params.matchLineY = 35.0;
params.exitY = 90.0;

% Lane IDs.
params.LEFT_LANE   = 1;
params.MIDDLE_LANE = 2;
params.RIGHT_LANE  = 3;

% Lane center x-positions.
params.laneX = [-params.laneWidth, 0, params.laneWidth];

% Safety metrics.
params.collisionRadius = 10.0;
params.nearMissRadius = 15.0;

% Baseline merge-search window.
params.mergeSearchAhead = 5.0;
params.mergeSearchBehind = 5.0;
params.unsafeYieldSpeed = 2.0;
params.yieldSpeedMargin = 2.0;
params.desiredGap = 14.0;
params.emergencyGap = 8.0;
params.followDetectRange = 100.0;

% CBF parameters.
params.cbfGamma = 1.0;
params.kCohesion = 0.35;
params.vMax = 24.0;

% Acceleration limits.
params.maxAccel = 3.0;
params.maxDecel = 10.0;
params.matchAccel = 6.0;

% Optional animation for debugging. Keep false for metric sweeps.
params.animate = false;
params.pauseTime = 0.02;

%% Define test scenarios

scenarios = makeTestScenarios(params);
controllers = {'baseline', 'cbf_supervised'};

%% Run all experiments

rows = {};

for s = 1:numel(scenarios)
    for c = 1:numel(controllers)

        controllerName = controllers{c};
        scenarioSpec = scenarios(s);

        fprintf('\nRunning scenario: %s | controller: %s\n', ...
            scenarioSpec.name, controllerName);

        result = runMergeSimulation(controllerName, scenarioSpec, params);

        rows(end+1,:) = { ... %#ok<SAGROW>
            scenarioSpec.name, ...
            controllerName, ...
            result.numVehicles, ...
            result.numExited, ...
            result.completionTime_s, ...
            result.collisionCount, ...
            result.nearMissCount, ...
            result.throughput_veh_per_min, ...
            result.qpInfeasibleCount, ...
            result.meanCBFIntervention_mps, ...
            result.maxCBFIntervention_mps, ...
            result.mergeCompletionMean_s, ...
            result.mergeCompletionLast_s};
    end
end

resultsTable = cell2table(rows, 'VariableNames', { ...
    'Scenario', ...
    'Controller', ...
    'NumVehicles', ...
    'NumExited', ...
    'CompletionTime_s', ...
    'CollisionCount', ...
    'NearMissCount', ...
    'Throughput_veh_per_min', ...
    'QPInfeasibleCount', ...
    'MeanCBFIntervention_mps', ...
    'MaxCBFIntervention_mps', ...
    'MergeCompletionMean_s', ...
    'MergeCompletionLast_s'});

disp(resultsTable);

writetable(resultsTable, 'merge_controller_metrics.csv');
fprintf('\nSaved metrics table to merge_controller_metrics.csv\n');


%% ------------------------------------------------------------------------
% Local functions
% -------------------------------------------------------------------------

function scenarios = makeTestScenarios(params)
%MAKETESTSCENARIOS Define the requested merge test cases.

    laneX = params.laneX;
    M = params.MIDDLE_LANE;
    R = params.RIGHT_LANE;

    % Scenario A:
    %   two cars in the middle lane, one car in the merge lane.
    scenarios(1).name = 'two_middle_one_merge';
    scenarios(1).cars = [ ...
        makeInitialCar('M1', M, laneX(M), -5,  12.0); ...
        makeInitialCar('M2', M, laneX(M), -35, 14.0); ...
        makeInitialCar('R1', R, laneX(R), -60, 18.0)];

    % Scenario B:
    %   two cars in the merge lane, one car in the middle lane.
    scenarios(2).name = 'two_merge_one_middle';
    scenarios(2).cars = [ ...
        makeInitialCar('M1', M, laneX(M), -15, 14.0); ...
        makeInitialCar('R1', R, laneX(R), -40, 18.0); ...
        makeInitialCar('R2', R, laneX(R), -75, 20.0)];
end


function car = makeInitialCar(name, originLane, x0, y0, cruiseSpeed)
%MAKEINITIALCAR Initial vehicle struct used by both controllers.

    car.name = name;

    car.originLane = originLane;
    car.currentLane = originLane;

    car.x = x0;
    car.y = y0;

    car.speed = cruiseSpeed;
    car.cruiseSpeed = cruiseSpeed;
    car.cmdSpeed = cruiseSpeed;

    car.yaw = 90;

    % Baseline FSM fields.
    car.mode = 'CRUISE';
    car.arrivedAtControlLine = false;
    car.arrivalTime = inf;
    car.hasDecision = false;
    car.yieldToID = 0;

    % Merge-supervisor fields.
    car.hasStartedMerge = originLane ~= 3;
    car.mergeStartY = inf;
    car.mergeBlocked = false;
    car.mergeBlockID = 0;

    % Metrics fields.
    car.hasExited = false;
    car.exitTime = inf;
    car.mergeCompleted = originLane ~= 3;
    car.mergeCompletionTime = inf;
end


function result = runMergeSimulation(controllerName, scenarioSpec, params)
%RUNMERGESIMULATION Simulate one scenario with one controller.

    cars = scenarioSpec.cars;
    numCars = numel(cars);
    numSteps = round(params.T / params.dt) + 1;

    simState.activeROW = 0;

    metrics = initializeMetrics(numCars, params);

    if params.animate
        [scenario, cars, ax] = initializeAnimation(cars, params); %#ok<ASGLU>
    else
        ax = [];
    end

    for k = 1:numSteps
        t = (k - 1) * params.dt;

        switch controllerName
            case 'baseline'
                [speedCommands, cars, simState] = baselineController(cars, simState, params, t);
                nominalSpeeds = nan(numCars, 1);
                safeSpeeds = speedCommands(:);
                qpFailedThisStep = false;

                % Apply acceleration limits for baseline.
                for i = 1:numCars
                    accelMode = cars(i).mode;
                    safeSpeeds(i) = applySpeedCommand(cars(i).speed, speedCommands(i), params, accelMode);
                end

            case 'cbf_supervised'
                [safeSpeeds, nominalSpeeds, qpFailedThisStep, cars] = ...
                    cbfSupervisedController(cars, params, t);

            otherwise
                error('Unknown controller: %s', controllerName);
        end

        oldCars = cars;

        % Integrate motion.
        for i = 1:numCars
            cars(i).speed = safeSpeeds(i);
            cars(i).y = cars(i).y + cars(i).speed * params.dt;

            [cars(i).x, cars(i).currentLane] = computeLanePosition(cars(i), params);

            dx = cars(i).x - oldCars(i).x;
            dy = cars(i).y - oldCars(i).y;

            if hypot(dx, dy) > 1e-8
                cars(i).yaw = atan2d(dy, dx);
            end
        end

        % Update event flags after motion.
        for i = 1:numCars
            if ~cars(i).hasExited && cars(i).y >= params.exitY
                cars(i).hasExited = true;
                cars(i).exitTime = t;
            end

            if cars(i).originLane == params.RIGHT_LANE && ...
               ~cars(i).mergeCompleted && ...
               cars(i).hasStartedMerge && ...
               cars(i).y >= params.matchLineY

                cars(i).mergeCompleted = true;
                cars(i).mergeCompletionTime = t;
            end
        end

        metrics = updateMetrics(metrics, cars, safeSpeeds, nominalSpeeds, ...
            qpFailedThisStep, controllerName, params, t);

        if params.animate
            updateAnimation(ax, cars, params, t, controllerName);
        end

        if all([cars.hasExited])
            break;
        end
    end

    result = finalizeMetrics(metrics, cars, controllerName, params);
end


function metrics = initializeMetrics(numCars, params)
%INITIALIZEMETRICS Create metric storage.

    metrics.numCars = numCars;

    metrics.collisionCount = 0;
    metrics.nearMissCount = 0;

    metrics.collisionActive = false(numCars, numCars);
    metrics.nearMissActive = false(numCars, numCars);

    metrics.qpInfeasibleCount = 0;

    metrics.interventionValues = [];

    metrics.activeMinDistance = inf;
    metrics.globalMinDistance = inf;

    metrics.numSteps = 0;
    metrics.finalTime = params.T;
end


function metrics = updateMetrics(metrics, cars, safeSpeeds, nominalSpeeds, ...
    qpFailedThisStep, controllerName, params, t)
%UPDATEMETRICS Update safety, QP, and intervention metrics.

    numCars = numel(cars);
    metrics.numSteps = metrics.numSteps + 1;
    metrics.finalTime = t;

    if qpFailedThisStep
        metrics.qpInfeasibleCount = metrics.qpInfeasibleCount + 1;
    end

    if strcmp(controllerName, 'cbf_supervised')
        intervention = abs(safeSpeeds(:) - nominalSpeeds(:));
        metrics.interventionValues = [metrics.interventionValues; intervention(:)]; %#ok<AGROW>
    end

    for i = 1:numCars-1
        for j = i+1:numCars

            pi = [cars(i).x, cars(i).y];
            pj = [cars(j).x, cars(j).y];
            d = norm(pi - pj);

            metrics.globalMinDistance = min(metrics.globalMinDistance, d);

            relevantPair = shouldApplyCollisionCBF(i, j, cars, params);

            if relevantPair
                metrics.activeMinDistance = min(metrics.activeMinDistance, d);

                isCollision = d < params.collisionRadius;
                isNearMiss = d >= params.collisionRadius && d < params.nearMissRadius;

                if isCollision && ~metrics.collisionActive(i,j)
                    metrics.collisionCount = metrics.collisionCount + 1;
                    metrics.collisionActive(i,j) = true;
                elseif ~isCollision
                    metrics.collisionActive(i,j) = false;
                end

                if isNearMiss && ~metrics.nearMissActive(i,j)
                    metrics.nearMissCount = metrics.nearMissCount + 1;
                    metrics.nearMissActive(i,j) = true;
                elseif ~isNearMiss
                    metrics.nearMissActive(i,j) = false;
                end
            else
                metrics.collisionActive(i,j) = false;
                metrics.nearMissActive(i,j) = false;
            end
        end
    end
end


function result = finalizeMetrics(metrics, cars, controllerName, params)
%FINALIZEMETRICS Convert raw logs into one result row.

    exitTimes = [cars.exitTime];
    exited = isfinite(exitTimes);
    numExited = sum(exited);

    if all(exited)
        completionTime = max(exitTimes);
    else
        completionTime = NaN;
    end

    if isnan(completionTime) || completionTime <= 0
        throughputTime = params.T;
    else
        throughputTime = completionTime;
    end

    throughput = numExited / (throughputTime / 60.0);

    mergeTimes = [cars.mergeCompletionTime];
    mergeTimes = mergeTimes(isfinite(mergeTimes));

    if isempty(mergeTimes)
        mergeMean = NaN;
        mergeLast = NaN;
    else
        mergeMean = mean(mergeTimes);
        mergeLast = max(mergeTimes);
    end

    if strcmp(controllerName, 'cbf_supervised') && ~isempty(metrics.interventionValues)
        meanIntervention = mean(metrics.interventionValues);
        maxIntervention = max(metrics.interventionValues);
    else
        meanIntervention = NaN;
        maxIntervention = NaN;
    end

    result.numVehicles = numel(cars);
    result.numExited = numExited;
    result.completionTime_s = completionTime;
    result.collisionCount = metrics.collisionCount;
    result.nearMissCount = metrics.nearMissCount;
    result.throughput_veh_per_min = throughput;
    result.qpInfeasibleCount = metrics.qpInfeasibleCount;
    result.meanCBFIntervention_mps = meanIntervention;
    result.maxCBFIntervention_mps = maxIntervention;
    result.mergeCompletionMean_s = mergeMean;
    result.mergeCompletionLast_s = mergeLast;
end


%% ------------------------------------------------------------------------
% Baseline controller
% -------------------------------------------------------------------------

function [speedCommands, cars, simState] = baselineController(cars, simState, params, t)
%BASELINECONTROLLER Rule-based baseline from earlier development.

    numCars = numel(cars);
    speedCommands = zeros(numCars, 1);

    % Clear ROW after it has created a downstream gap.
    if simState.activeROW ~= 0
        activeID = simState.activeROW;
        if cars(activeID).y >= params.matchLineY + params.desiredGap
            simState.activeROW = 0;
        end
    end

    % Detect control line arrivals.
    for i = 1:numCars
        if ~cars(i).arrivedAtControlLine
            yNow = cars(i).y;
            yNext = cars(i).y + cars(i).speed * params.dt;

            if yNow >= params.controlLineY || yNext >= params.controlLineY
                cars(i).arrivedAtControlLine = true;

                if cars(i).speed > 1e-6
                    frac = (params.controlLineY - yNow) / max(cars(i).speed * params.dt, 1e-6);
                    frac = min(max(frac, 0), 1);
                    cars(i).arrivalTime = t + frac * params.dt;
                else
                    cars(i).arrivalTime = t;
                end
            end
        end
    end

    % Assign ROW/yield decisions.
    candidates = find([cars.arrivedAtControlLine] & ~[cars.hasDecision]);

    if ~isempty(candidates)
        if simState.activeROW == 0
            winnerID = chooseRightOfWay(candidates, cars);

            cars(winnerID).mode = 'ROW';
            cars(winnerID).hasDecision = true;

            if cars(winnerID).originLane == params.RIGHT_LANE
                cars(winnerID).hasStartedMerge = true;
                cars(winnerID).mergeStartY = max(cars(winnerID).y, params.controlLineY);
            end

            simState.activeROW = winnerID;

            for idx = 1:numel(candidates)
                id = candidates(idx);
                if id ~= winnerID
                    cars(id).mode = 'YIELD';
                    cars(id).hasDecision = true;
                    cars(id).yieldToID = winnerID;
                end
            end
        else
            for idx = 1:numel(candidates)
                id = candidates(idx);
                cars(id).mode = 'YIELD';
                cars(id).hasDecision = true;
                cars(id).yieldToID = simState.activeROW;
            end
        end
    end

    % Merge admission for right-lane yielding vehicles.
    for i = 1:numCars
        isRightLaneYielding = strcmp(cars(i).mode, 'YIELD') && cars(i).originLane == params.RIGHT_LANE;

        if isRightLaneYielding && ~cars(i).hasStartedMerge
            [isSafe, ~] = isTargetLaneClearWindow(i, cars, params.MIDDLE_LANE, params);

            if isSafe
                cars(i).hasStartedMerge = true;
                cars(i).mergeStartY = max(cars(i).y, params.controlLineY);
                cars(i).mergeBlocked = false;
                cars(i).mergeBlockID = 0;
            else
                cars(i).hasStartedMerge = false;
                cars(i).mergeStartY = inf;
                cars(i).mergeBlocked = true;
            end
        end
    end

    % Transition yielded merge cars to MATCH after match line.
    for i = 1:numCars
        if strcmp(cars(i).mode, 'YIELD') && ...
           cars(i).hasStartedMerge && ...
           cars(i).y >= params.matchLineY
            cars(i).mode = 'MATCH';
        end
    end

    % Compute speed commands.
    for i = 1:numCars
        switch cars(i).mode
            case 'CRUISE'
                speedCommands(i) = cars(i).cruiseSpeed;

            case 'ROW'
                speedCommands(i) = cars(i).cruiseSpeed;

            case 'YIELD'
                speedCommands(i) = computeBaselineYieldSpeed(i, cars, params);

            case 'MATCH'
                speedCommands(i) = computeMatchSpeed(i, cars, params);

            otherwise
                speedCommands(i) = cars(i).cruiseSpeed;
        end
    end

    % Conservative same-lane following for baseline.
    for i = 1:numCars
        leadID = findLeadVehicleInEffectiveLane(i, cars, getEffectiveLane(cars(i), params), params);

        if leadID ~= 0
            gap = cars(leadID).y - cars(i).y;
            if gap < params.desiredGap
                speedCommands(i) = min(speedCommands(i), ...
                    max(cars(leadID).speed - params.yieldSpeedMargin, 0));
            end
        end
    end
end


function winnerID = chooseRightOfWay(candidateIDs, cars)
%CHOOSERIGHTOFWAY Baseline priority rule.

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
    sortData = [arrivalTimes(:), lanes(:), candidateIDs(:)];
    sorted = sortrows(sortData, [1, 2]);
    winnerID = sorted(1, 3);
end


function speedCommand = computeBaselineYieldSpeed(i, cars, params)
%COMPUTEBASELINEYIELDSPEED Rule-based yield command.

    if cars(i).yieldToID == 0
        speedCommand = cars(i).cruiseSpeed;
        return;
    end

    rowID = cars(i).yieldToID;
    rowSpeed = cars(rowID).speed;
    rowY = cars(rowID).y;
    egoY = cars(i).y;
    gap = rowY - egoY;

    % If this right-lane car has not been cleared to merge, slow to 2 m/s.
    if cars(i).originLane == params.RIGHT_LANE && ~cars(i).hasStartedMerge
        speedCommand = params.unsafeYieldSpeed;

        % Do not cross the match line while still not merged.
        distanceToMatch = params.matchLineY - egoY;
        if distanceToMatch <= 0
            speedCommand = 0;
        else
            speedCommand = min(speedCommand, distanceToMatch / params.dt);
        end
        return;
    end

    % If merging is allowed, yield by being slower than ROW vehicle.
    speedCommand = min(cars(i).cruiseSpeed, max(rowSpeed - params.yieldSpeedMargin, 0));

    if gap < params.emergencyGap
        speedCommand = max(rowSpeed - 2*params.yieldSpeedMargin, 0);
    end

    if gap <= 0
        speedCommand = 0;
    end
end


%% ------------------------------------------------------------------------
% CBF supervised controller
% -------------------------------------------------------------------------

function [safeSpeeds, nominalSpeeds, qpFailedThisStep, cars] = cbfSupervisedController(cars, params, t) %#ok<INUSD>
%CBFSUPERVISEDCONTROLLER Supervised CBF-QP controller.

    numCars = numel(cars);

    % Minimal merge-admissibility supervisor.
    for i = 1:numCars
        if cars(i).originLane == params.RIGHT_LANE && ...
           ~cars(i).hasStartedMerge && ...
           cars(i).y >= params.controlLineY

            [isSafe, blockingID] = isTargetLaneClearWindow(i, cars, params.MIDDLE_LANE, params);

            if isSafe
                cars(i).hasStartedMerge = true;
                cars(i).mergeStartY = max(cars(i).y, params.controlLineY);
                cars(i).mergeBlocked = false;
                cars(i).mergeBlockID = 0;
            else
                cars(i).hasStartedMerge = false;
                cars(i).mergeStartY = inf;
                cars(i).mergeBlocked = true;
                cars(i).mergeBlockID = blockingID;
            end
        end
    end

    [safeSpeeds, nominalSpeeds, qpFailedThisStep] = solveCBFSpeeds(cars, params);
end


function [safeSpeeds, nominalSpeeds, qpFailedThisStep] = solveCBFSpeeds(cars, params)
%SOLVECBFSPEEDS Centralized speed-level CBF-QP.

    numCars = numel(cars);

    positions = zeros(numCars, 2);
    pathDerivatives = zeros(numCars, 2);

    for i = 1:numCars
        positions(i,:) = [cars(i).x, cars(i).y];
        pathDerivatives(i,:) = getPathDerivative(cars(i), params);
    end

    nominalSpeeds = computeNominalSpeeds(cars, params);

    H = eye(numCars);
    f = -nominalSpeeds(:);

    A = [];
    b = [];

    for i = 1:numCars-1
        for j = i+1:numCars

            if ~shouldApplyCollisionCBF(i, j, cars, params)
                continue;
            end

            p_i = positions(i,:).';
            p_j = positions(j,:).';

            q_i = pathDerivatives(i,:).';
            q_j = pathDerivatives(j,:).';

            r_ij = p_i - p_j;
            h_ij = dot(r_ij, r_ij) - params.collisionRadius^2;

            % h_dot + gamma*h >= 0.
            % h_dot = 2*r'*(q_i*v_i - q_j*v_j).
            % Quadprog form A*v <= b.
            Arow = zeros(1, numCars);
            Arow(i) = -2 * (r_ij.' * q_i);
            Arow(j) =  2 * (r_ij.' * q_j);
            brow = params.cbfGamma * h_ij;

            A = [A; Arow]; %#ok<AGROW>
            b = [b; brow]; %#ok<AGROW>
        end
    end

    lb = zeros(numCars, 1);
    ub = params.vMax * ones(numCars, 1);

    for i = 1:numCars
        lb(i) = max(0, cars(i).speed - params.maxDecel * params.dt);
        ub(i) = min(params.vMax, cars(i).speed + params.maxAccel * params.dt);
    end

    options = optimset('Display', 'off');

    [v, ~, exitflagInitial] = quadprog(H, f, A, b, [], [], lb, ub, [], options);

    qpFailedThisStep = exitflagInitial <= 0;

    if exitflagInitial <= 0
        % Fallback without acceleration bounds.
        [v, ~, exitflagFallback] = quadprog( ...
            H, f, A, b, [], [], zeros(numCars,1), params.vMax*ones(numCars,1), [], options);

        if exitflagFallback <= 0
            warning('CBF-QP infeasible even after fallback. Commanding zero speeds.');
            v = zeros(numCars, 1);
        end
    end

    safeSpeeds = v(:);
end


function nominalSpeeds = computeNominalSpeeds(cars, params)
%COMPUTENOMINALSPEEDS Cruise + same-lane cohesion + supervisor slowing.

    numCars = numel(cars);
    nominalSpeeds = zeros(numCars, 1);

    for i = 1:numCars

        if cars(i).mergeBlocked && cars(i).mergeBlockID ~= 0
            blockID = cars(i).mergeBlockID;
            nominalSpeeds(i) = 0.5 * cars(blockID).speed;

            % Do not let a blocked merge vehicle drive past the match line.
            if cars(i).originLane == params.RIGHT_LANE && ~cars(i).hasStartedMerge
                distanceToMatch = params.matchLineY - cars(i).y;
                if distanceToMatch <= 0
                    nominalSpeeds(i) = 0;
                else
                    nominalSpeeds(i) = min(nominalSpeeds(i), distanceToMatch / params.dt);
                end
            end

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


%% ------------------------------------------------------------------------
% Shared vehicle behavior helpers
% -------------------------------------------------------------------------

function newSpeed = applySpeedCommand(currentSpeed, commandSpeed, params, mode)
%APPLYSPEEDCOMMAND Apply acceleration/deceleration limits.

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


function speedCommand = computeMatchSpeed(i, cars, params)
%COMPUTEMATCHSPEED Match current speed of nearest vehicle ahead.

    leadID = findLeadVehicleInEffectiveLane(i, cars, getEffectiveLane(cars(i), params), params);

    if leadID == 0
        speedCommand = cars(i).cruiseSpeed;
        return;
    end

    gap = cars(leadID).y - cars(i).y;
    speedCommand = cars(leadID).speed;

    if gap < params.desiredGap
        speedCommand = max(cars(leadID).speed - params.yieldSpeedMargin, 0);
    end
end


function [x, currentLane] = computeLanePosition(car, params)
%COMPUTELANEPOSITION Compute x-position based on y-position and merge state.

    laneX = params.laneX;

    if car.originLane ~= params.RIGHT_LANE
        x = laneX(car.originLane);
        currentLane = car.originLane;
        return;
    end

    if ~car.hasStartedMerge || car.y <= params.controlLineY
        x = laneX(params.RIGHT_LANE);
        currentLane = params.RIGHT_LANE;
        return;
    end

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

        x = (1 - s) * laneX(params.RIGHT_LANE) + s * laneX(params.MIDDLE_LANE);
        currentLane = params.RIGHT_LANE;
        return;
    end

    x = laneX(params.MIDDLE_LANE);
    currentLane = params.MIDDLE_LANE;
end


function q = getPathDerivative(car, params)
%GETPATHDERIVATIVE Return p_dot per unit y-speed.

    laneX = params.laneX;

    if car.originLane == params.RIGHT_LANE && ...
       car.hasStartedMerge && ...
       car.y < params.matchLineY

        dxdy = (laneX(params.MIDDLE_LANE) - laneX(params.RIGHT_LANE)) / ...
               (params.matchLineY - params.controlLineY);
        q = [dxdy, 1];
    else
        q = [0, 1];
    end
end


function effectiveLane = getEffectiveLane(car, params)
%GETEFFECTIVELANE Lane used for following/cohesion/collision relevance.

    if car.originLane == params.RIGHT_LANE && car.hasStartedMerge
        effectiveLane = params.MIDDLE_LANE;
    else
        effectiveLane = car.currentLane;
    end
end


function leadID = findLeadVehicleInEffectiveLane(i, cars, laneID, params)
%FINDLEADVEHICLEINEFFECTIVELANE Nearest vehicle ahead in same effective lane.

    leadID = 0;
    bestGap = inf;

    for j = 1:numel(cars)
        if j == i
            continue;
        end

        if getEffectiveLane(cars(j), params) ~= laneID
            continue;
        end

        gap = cars(j).y - cars(i).y;

        if gap > 0 && gap < bestGap && gap < params.followDetectRange
            bestGap = gap;
            leadID = j;
        end
    end
end


function [isClear, blockingID] = isTargetLaneClearWindow(i, cars, targetLane, params)
%ISTARGETLANECLEARWINDOW Check target lane from behind to ahead.
%
% Unsafe if another vehicle in the target lane satisfies:
%   -mergeSearchBehind <= y_j - y_i <= mergeSearchAhead

    egoY = cars(i).y;

    isClear = true;
    blockingID = 0;

    for j = 1:numel(cars)
        if j == i
            continue;
        end

        if getEffectiveLane(cars(j), params) ~= targetLane
            continue;
        end

        relativeY = cars(j).y - egoY;

        if relativeY >= -params.mergeSearchBehind && relativeY <= params.mergeSearchAhead
            isClear = false;
            blockingID = j;
            return;
        end
    end
end


function applyCBF = shouldApplyCollisionCBF(i, j, cars, params)
%SHOULDAPPLYCOLLISIONCBF Decide whether pairwise collision avoidance is active.

    iIsMerging = isVehicleMerging(cars(i), params);
    jIsMerging = isVehicleMerging(cars(j), params);

    sameEffectiveLane = getEffectiveLane(cars(i), params) == ...
                        getEffectiveLane(cars(j), params);

    applyCBF = iIsMerging || jIsMerging || sameEffectiveLane;
end


function isMerging = isVehicleMerging(car, params)
%ISVEHICLEMERGING True if actively transitioning from lane 3 to lane 2.

    isMerging = car.originLane == params.RIGHT_LANE && ...
                car.hasStartedMerge && ...
                car.y >= params.controlLineY && ...
                car.y < params.matchLineY;
end


%% ------------------------------------------------------------------------
% Optional animation helpers
% -------------------------------------------------------------------------

function [scenario, cars, ax] = initializeAnimation(cars, params)
%INITIALIZEANIMATION Optional AV Toolbox visualization.

    scenario = drivingScenario('SampleTime', params.dt, 'StopTime', params.T);

    upstreamRoadCenters = [0 -120 0; 0 params.matchLineY 0];
    upstreamLanes = lanespec(3, 'Width', params.laneWidth);
    road(scenario, upstreamRoadCenters, 'Lanes', upstreamLanes);

    downstreamCenterX = -params.laneWidth / 2;
    downstreamRoadCenters = [downstreamCenterX params.matchLineY 0; downstreamCenterX 140 0];
    downstreamLanes = lanespec(2, 'Width', params.laneWidth);
    road(scenario, downstreamRoadCenters, 'Lanes', downstreamLanes);

    for i = 1:numel(cars)
        cars(i).obj = vehicle(scenario, ...
            'ClassID', 1, ...
            'Name', cars(i).name, ...
            'Position', [cars(i).x, cars(i).y, 0], ...
            'Yaw', 90);
    end

    fig = figure;
    ax = axes('Parent', fig);
end


function updateAnimation(ax, cars, params, t, controllerName)
%UPDATEANIMATION Optional single-figure animation.

    persistent scenarioInitialized
    %#ok<NASGU>

    for i = 1:numel(cars)
        if isfield(cars(i), 'obj')
            cars(i).obj.Position = [cars(i).x, cars(i).y, 0];
            cars(i).obj.Yaw = cars(i).yaw;
        end
    end

    cla(ax);

    % Plotting the drivingScenario object from here is awkward because it is
    % local to initializeAnimation. Keep this optional animation simple.
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 2);

    laneX = params.laneX;

    % Lane centerlines.
    plot(ax, [laneX(1), laneX(1)], [-120, params.matchLineY], 'k:');
    plot(ax, [laneX(2), laneX(2)], [-120, 140], 'k:');
    plot(ax, [laneX(3), laneX(3)], [-120, params.matchLineY], 'k:');
    plot(ax, [laneX(3), laneX(2)], [params.controlLineY, params.matchLineY], 'm--', 'LineWidth', 1.5);

    % Lines.
    plot(ax, [laneX(1)-params.laneWidth/2, laneX(3)+params.laneWidth/2], ...
        [params.controlLineY, params.controlLineY], 'r-', 'LineWidth', 2);
    plot(ax, [laneX(1)-params.laneWidth/2, laneX(3)+params.laneWidth/2], ...
        [params.matchLineY, params.matchLineY], 'k-', 'LineWidth', 2);

    for i = 1:numel(cars)
        plot(ax, cars(i).x, cars(i).y, 'o', 'MarkerSize', 8, 'LineWidth', 2);
        text(ax, cars(i).x + 0.5, cars(i).y, sprintf('%s %.1f m/s', cars(i).name, cars(i).speed));
    end

    title(ax, sprintf('%s | t = %.1f s', controllerName, t));
    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');
    xlim(ax, [-15, 12]);
    ylim(ax, [-90, 100]);

    hold(ax, 'off');
    drawnow limitrate;
    pause(params.pauseTime);
end
