classdef StateLogBuilder < handle
    % STATELOGBUILDER Builds the per-step simulation telemetry stateLog.
    % Owns stateLog allocation, per-step channel updates, final
    % truncation, and replay reference-channel assembly, so the channel
    % set, field order, units, and lean-mode dropping logic are defined
    % in one place. Extracted verbatim from lts.simulation.Simulator;
    % exported channel names must stay byte-identical for the CSV
    % exporter and correlation tooling.
    %
    % Telemetry modes mirror Simulator.telemetryMode: only "lean" is
    % special-cased (drops the per-corner suspension/tire channels);
    % every other value logs the full channel set.
    %
    % Usage:
    %   builder = lts.telemetry.StateLogBuilder(vm, telemetryMode);
    %   builder.beginRun(nSteps, trackData, isFreeReference);
    %   builder.logStep(k, prevState, newState, input, forces);
    %   stateLog = builder.finish();

    properties (SetAccess = private)
        stateLog = struct()
        loggedSteps = 0
    end

    properties (Access = private)
        vehicleManager
        lean = false
        trackData = []
        freeReferenceMode = false
    end

    methods
        function obj = StateLogBuilder(vehicleManager, telemetryMode)
            obj.vehicleManager = vehicleManager;
            if nargin < 2 || isempty(telemetryMode)
                telemetryMode = "full";
            end
            obj.lean = lower(string(telemetryMode)) == "lean";
        end

        function beginRun(obj, nSteps, trackData, freeReferenceMode)
            % Allocate the log for a new run and remember per-run context
            % used by the per-step track-limit channels.
            if nargin < 3
                trackData = [];
            end
            if nargin < 4
                freeReferenceMode = false;
            end
            obj.stateLog = lts.telemetry.StateLogBuilder.createStateLog( ...
                nSteps, obj.lean);
            obj.loggedSteps = 0;
            obj.trackData = trackData;
            obj.freeReferenceMode = logical(freeReferenceMode);
        end

        function logStep(obj, k, prevState, newState, input, forces)
            % Record one simulated step. k is the 1-based step index,
            % prevState is the step's input state (driver-source
            % defaults), newState and forces are the step() results.
            stateLog = obj.stateLog;
            trackData = obj.trackData;

            inputSourceDist   = localGetField(input, 'sourceDistance',      prevState.s);
            inputSourceTime   = localGetField(input, 'sourceTime',          prevState.time);
            inputTargetSpeed  = localGetField(input, 'targetSpeed',         NaN);
            inputAxRef        = localGetField(input, 'axRef',               NaN);
            inputTargetLatErr = localGetField(input, 'targetLateralError',  NaN);
            inputLineCurv     = localGetField(input, 'lineCurvature',       NaN);
            inputSpeedError   = localGetField(input, 'speedError',          NaN);

            stateLog.time(k)        = newState.time;
            stateLog.s(k)           = newState.s;
            stateLog.controlS(k)    = inputSourceDist;
            stateLog.x(k)           = newState.x;
            stateLog.y(k)           = newState.y;
            stateLog.yaw(k)         = newState.yaw;
            stateLog.vx(k)          = newState.vx;
            stateLog.vy(k)          = newState.vy;
            stateLog.bodySlipAngle(k) = newState.bodySlipAngle;
            stateLog.speed(k)       = newState.speed;
            stateLog.speedKmh(k)    = newState.speed * 3.6;
            stateLog.controlTime(k) = inputSourceTime;
            stateLog.ax(k)          = newState.ax;
            stateLog.ay(k)          = newState.ay;
            stateLog.frontAxleAy(k) = newState.frontAxleAy;
            stateLog.rearAxleAy(k)  = newState.rearAxleAy;
            stateLog.yawRate(k)     = newState.yawRate;
            stateLog.yawAccel(k)    = newState.yawAccel;
            stateLog.refS(k)        = newState.refS;
            stateLog.refHeading(k)  = newState.refHeading;
            stateLog.refCurvature(k) = newState.refCurvature;
            stateLog.lateralError(k) = newState.lateralError;
            stateLog.onTrack(k)     = newState.onTrack;
            if obj.freeReferenceMode
                stateLog.trackWidth(k) = 0;
                stateLog.trackLimitMargin(k) = 0;
            else
                refIdxStep = find(trackData.arcLen <= newState.refS, 1, 'last');
                if isempty(refIdxStep)
                    refIdxStep = 1;
                end
                refIdxStep = max(1, min(refIdxStep, ...
                    numel(trackData.trackLeftHalfWidth)));
                % Track width remains a scalar telemetry channel.
                stateLog.trackWidth(k) = ...
                    trackData.trackLeftHalfWidth(refIdxStep) + ...
                    trackData.trackRightHalfWidth(refIdxStep);
                [localLeft, localRight] = ...
                    lts.simulation.TrackReference.sideHalfWidthsAt( ...
                    trackData, refIdxStep);
                stateLog.trackLimitMargin(k) = ...
                    lts.simulation.TrackReference.sideMargin( ...
                    localLeft, localRight, newState.lateralError, ...
                    trackData.trackHalfWidth);
            end
            stateLog.throttle(k)    = input.throttle;
            stateLog.brake(k)       = forces.brake;
            stateLog.brakeRequested(k) = forces.brakeCommand;
            stateLog.brakePressureMode(k) = forces.brakePressureMode;
            stateLog.brakePressureFrontBar(k) = forces.brakePressureFrontBar;
            stateLog.brakePressureRearBar(k) = forces.brakePressureRearBar;
            stateLog.steer(k)       = input.steer;
            stateLog.targetSpeed(k) = inputTargetSpeed;
            stateLog.axRef(k)       = inputAxRef;
            stateLog.targetLateralError(k) = inputTargetLatErr;
            stateLog.lineCurvature(k) = inputLineCurv;
            if isfinite(inputSpeedError)
                stateLog.speedError(k) = inputSpeedError;
            elseif isfinite(inputTargetSpeed)
                stateLog.speedError(k) = prevState.speed - inputTargetSpeed;
            end
            stateLog.curvature(k)   = newState.refCurvature;
            stateLog.heading(k)     = newState.heading;
            stateLog.F_downforce(k) = forces.F_downforce;
            stateLog.F_drag(k)      = forces.F_drag;
            stateLog.F_drive(k)     = forces.F_drive;
            stateLog.F_brake(k)     = forces.F_brake;
            stateLog.F_tire_long(k) = forces.F_tire_long;
            stateLog.F_tire_lat(k)  = forces.F_tire_lat;
            stateLog.yawMoment(k)   = forces.yawMoment;
            stateLog.rollResistance(k) = forces.rollResistance;
            stateLog.F_brake_front(k) = forces.F_brake_front;
            stateLog.F_brake_rear(k) = forces.F_brake_rear;
            stateLog.F_brake_FL(k)  = forces.F_brake_FL;
            stateLog.F_brake_FR(k)  = forces.F_brake_FR;
            stateLog.F_brake_RL(k)  = forces.F_brake_RL;
            stateLog.F_brake_RR(k)  = forces.F_brake_RR;
            stateLog.brakeGrip_FL(k) = forces.brakeGrip_FL;
            stateLog.brakeGrip_FR(k) = forces.brakeGrip_FR;
            stateLog.brakeGrip_RL(k) = forces.brakeGrip_RL;
            stateLog.brakeGrip_RR(k) = forces.brakeGrip_RR;
            stateLog.driveTorqueTotal(k) = forces.driveTorqueTotal;
            stateLog.driveTorque_RL(k) = forces.driveTorque_RL;
            stateLog.driveTorque_RR(k) = forces.driveTorque_RR;
            stateLog.brakeTorque_FL(k) = forces.brakeTorque_FL;
            stateLog.brakeTorque_FR(k) = forces.brakeTorque_FR;
            stateLog.brakeTorque_RL(k) = forces.brakeTorque_RL;
            stateLog.brakeTorque_RR(k) = forces.brakeTorque_RR;
            stateLog.motorRPM(k)    = forces.motorRPM;
            stateLog.motorTorque(k) = forces.motorTorque;
            stateLog.motorTorqueRequested(k) = forces.motorTorqueRequested;
            stateLog.motorTorquePowerLimitNm(k) = forces.motorTorquePowerLimitNm;
            stateLog.motorTorquePowerLimitActive(k) = forces.motorTorquePowerLimitActive;
            stateLog.wheelTorque(k) = forces.wheelTorque;
            stateLog.packVoltageV(k) = forces.packVoltageV;
            stateLog.packCurrentA(k) = forces.packCurrentA;
            stateLog.packPowerW(k) = forces.packPowerW;
            stateLog.drivenWheelRPM(k) = forces.drivenWheelRPM;
            stateLog.rpmLimitActive(k) = forces.rpmLimitActive;
            stateLog.pitchAngle(k)  = newState.pitchAngle;
            stateLog.rollAngle(k)   = newState.rollAngle;
            stateLog.rollRate(k)    = newState.rollRate;
            stateLog.frontRollAngle(k) = newState.frontRollAngle;
            stateLog.rearRollAngle(k)  = newState.rearRollAngle;
            stateLog.frontRollRate(k)  = newState.frontRollRate;
            stateLog.rearRollRate(k)   = newState.rearRollRate;
            stateLog.twistAngle(k)     = newState.twistAngle;
            stateLog.twistRate(k)      = newState.twistRate;
            stateLog.rideHeight(k)  = newState.rideHeight;
            stateLog.aeroFz_front(k) = forces.aeroFz_front;
            stateLog.aeroFz_rear(k)  = forces.aeroFz_rear;
            stateLog = localLogCornerTelemetry( ...
                stateLog, k, obj.vehicleManager, obj.lean);

            obj.stateLog = stateLog;
            obj.loggedSteps = k;
        end

        function stateLog = finish(obj)
            % Truncate every channel to the logged step count.
            fields = fieldnames(obj.stateLog);
            for i = 1:numel(fields)
                obj.stateLog.(fields{i}) = obj.stateLog.(fields{i})(1:obj.loggedSteps);
            end
            stateLog = obj.stateLog;
        end
    end

    methods (Static)
        function stateLog = createStateLog(n, lean)
            % Allocate the stateLog channels. Field order defines CSV
            % column order; keep it stable.
            zeroFields = { ...
                'time','s','controlS','x','y','yaw','vx','vy','bodySlipAngle', ...
                'speed','speedKmh','controlTime','ax','ay','frontAxleAy','rearAxleAy', ...
                'yawRate','yawAccel','refS','refHeading','refCurvature','lateralError', ...
                'trackWidth','trackLimitMargin','throttle','brake','brakeRequested','steer', ...
                'curvature','heading','F_downforce','F_drag','F_drive','F_brake', ...
                'F_tire_long','F_tire_lat','yawMoment','rollResistance','F_brake_front', ...
                'F_brake_rear','F_brake_FL','F_brake_FR','F_brake_RL','F_brake_RR', ...
                'brakeGrip_FL','brakeGrip_FR','brakeGrip_RL','brakeGrip_RR', ...
                'driveTorqueTotal','driveTorque_RL','driveTorque_RR','brakeTorque_FL', ...
                'brakeTorque_FR','brakeTorque_RL','brakeTorque_RR','motorRPM','motorTorque', ...
                'motorTorqueRequested','wheelTorque','drivenWheelRPM','pitchAngle','rollAngle', ...
                'rollRate','frontRollAngle','rearRollAngle','frontRollRate','rearRollRate', ...
                'twistAngle','twistRate','rideHeight','aeroFz_front','aeroFz_rear', ...
                'tireSpeed_FL','tireSpeed_FR','tireSpeed_RL','tireSpeed_RR'};
            nanFields = { ...
                'brakePressureFrontBar','brakePressureRearBar','targetSpeed','axRef', ...
                'targetLateralError','lineCurvature','speedError','motorTorquePowerLimitNm', ...
                'packVoltageV','packCurrentA','packPowerW'};
            logicalFields = { ...
                'onTrack','brakePressureMode','motorTorquePowerLimitActive','rpmLimitActive'};

            stateLog = struct();
            for i = 1:numel(zeroFields)
                stateLog.(zeroFields{i}) = zeros(n, 1);
            end
            for i = 1:numel(nanFields)
                stateLog.(nanFields{i}) = NaN(n, 1);
            end
            for i = 1:numel(logicalFields)
                stateLog.(logicalFields{i}) = false(n, 1);
            end

            if lean
                return;
            end
            corners = {'FL','FR','RL','RR'};
            cornerFields = { ...
                'Fz','suspensionForce','antiRollBarForce','suspensionDemand','tireDeflection', ...
                'damperPos','damperVel','sprungPosition','unsprungPosition','sprungVelocity', ...
                'unsprungVelocity','wheelTravel','camber','toe','wheelSteer','slipAngle', ...
                'slipRatio','peakMu','tireUtilization','omega','tireFx','tireFy'};
            for i = 1:numel(cornerFields)
                for j = 1:numel(corners)
                    stateLog.([cornerFields{i} '_' corners{j}]) = zeros(n, 1);
                end
            end
        end

        function stateLog = addReplayReferenceChannels(stateLog, profile)
            % Append replay reference channels interpolated onto the
            % log's control-time (or time) axis.
            if isempty(stateLog.time)
                return;
            end

            if isfield(stateLog, 'controlTime')
                queryTime = stateLog.controlTime(:);
            else
                queryTime = stateLog.time(:);
            end

            channels = { ...
                'replayThrottle','throttle'; 'replayBrake','brake'; ...
                'replaySteer','steer'; 'replaySpeed','speed'; ...
                'replayYawRate','yawRate'};
            for i = 1:size(channels, 1)
                stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                    profile.time, profile.(channels{i, 2}), queryTime);
            end
            if profile.hasBrakePressure()
                channels = { ...
                    'replayBrakePressureFrontBar','brakePressureFrontBar'; ...
                    'replayBrakePressureRearBar','brakePressureRearBar'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
            end
            optionalChannels = { ...
                profile.hasRegenTorque(),'replayRegenTorqueNm','regenTorqueNm'; ...
                profile.hasMotorTorqueCommand(),'replayMotorTorqueCommandNm','motorTorqueCommandNm'; ...
                profile.hasMotorTorqueDelivered(),'replayMotorTorqueDeliveredNm','motorTorqueDeliveredNm'; ...
                profile.hasMotorRpm(),'replayMotorRpm','motorRpm'};
            for i = 1:size(optionalChannels, 1)
                if optionalChannels{i, 1}
                    stateLog.(optionalChannels{i, 2}) = localInterpProfileChannel( ...
                        profile.time, profile.(optionalChannels{i, 3}), queryTime);
                end
            end
            if profile.hasPackPower()
                channels = { ...
                    'replayPackVoltageV','packVoltageV'; ...
                    'replayPackCurrentA','packCurrentA'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
                stateLog.replayPackPowerW = ...
                    stateLog.replayPackVoltageV .* stateLog.replayPackCurrentA;
            end
            if profile.hasWheelSpeeds()
                corners = {'FL','FR','RL','RR'};
                for i = 1:numel(corners)
                    corner = corners{i};
                    replayField = ['replayWheelSpeed' corner];
                    simulatedField = ['tireSpeed_' corner];
                    stateLog.(replayField) = localInterpProfileChannel( ...
                        profile.time, profile.(['wheelSpeed' corner]), queryTime);
                    if isfield(stateLog, simulatedField)
                        stateLog.(['wheelSpeedError' corner]) = ...
                            stateLog.(simulatedField) - stateLog.(replayField);
                    end
                end
            end
            if profile.hasLatAccel()
                channels = { ...
                    'replayLatAccelG','latAccelG'; ...
                    'replayFrontLatAccelG','frontLatAccelG'; ...
                    'replayRearLatAccelG','rearLatAccelG'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
            end
            if profile.hasLongAccel()
                channels = { ...
                    'replayLongAccelG','longAccelG'; ...
                    'replayFrontLongAccelG','frontLongAccelG'; ...
                    'replayRearLongAccelG','rearLongAccelG'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
            end
        end
    end
end

function stateLog = localLogCornerTelemetry(stateLog, step, vm, lean)
corners = {'FL','FR','RL','RR'};
suspensionCorners = {'frontLeft','frontRight','rearLeft','rearRight'};
suspensionFields = { ...
    'Fz','tireNormalForce'; 'suspensionForce','suspensionForce'; ...
    'antiRollBarForce','antiRollBarForce'; 'suspensionDemand','demandedLoad'; ...
    'tireDeflection','tireDeflection'; 'damperPos','damperPosition'; ...
    'damperVel','damperVelocity'; 'sprungPosition','sprungPosition'; ...
    'unsprungPosition','unsprungPosition'; 'sprungVelocity','sprungVelocity'; ...
    'unsprungVelocity','unsprungVelocity'; 'wheelTravel','wheelTravel'; ...
    'camber','camberAngle'; 'toe','toeAngle'; 'wheelSteer','steerAngle'};
tireFields = { ...
    'slipAngle','slipAngle'; 'slipRatio','slipRatio'; 'peakMu','peakMu'; ...
    'omega','angularVelocity'; 'tireFx','Fx'; 'tireFy','Fy'; ...
    'relaxedFz','relaxedNormalLoad'};

for j = 1:numel(corners)
    corner = corners{j};
    tire = vm.tire.(corner);
    stateLog.(['tireSpeed_' corner])(step) = ...
        abs(tire.angularVelocity * tire.wheelRadius);
    if lean
        continue;
    end

    suspension = vm.suspension.(suspensionCorners{j}).state;
    for i = 1:size(suspensionFields, 1)
        stateLog.([suspensionFields{i, 1} '_' corner])(step) = ...
            suspension.(suspensionFields{i, 2});
    end
    for i = 1:size(tireFields, 1)
        stateLog.([tireFields{i, 1} '_' corner])(step) = ...
            tire.(tireFields{i, 2});
    end
    capacity = max(tire.peakMu, 0) * max(tire.normalForce, 0);
    utilization = 0;
    if capacity > eps
        utilization = hypot(tire.Fx, tire.Fy) / capacity;
        if ~isfinite(utilization)
            utilization = 0;
        end
    end
    stateLog.(['tireUtilization_' corner])(step) = utilization;
end
end

function value = localGetField(s, fieldName, defaultValue)
% Same contract as the Simulator helper: fall back on missing AND
% empty values (lts.util.fieldOrDefault only covers missing).
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
    if isempty(value)
        value = defaultValue;
    end
else
    value = defaultValue;
end
end

function values = localInterpProfileChannel(axis, channel, query)
axis = double(axis(:));
channel = double(channel(:));
query = double(query(:));

keep = isfinite(axis) & isfinite(channel);
axis = axis(keep);
channel = channel(keep);

if isempty(axis)
    values = NaN(size(query));
elseif isscalar(axis)
    values = repmat(channel(1), size(query));
else
    [axis, uniqueIdx] = unique(axis, 'stable');
    channel = channel(uniqueIdx);
    query = max(axis(1), min(axis(end), query));
    values = interp1(axis, channel, query, 'linear');
end
end
