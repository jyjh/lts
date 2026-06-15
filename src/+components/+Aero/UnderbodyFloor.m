classdef UnderbodyFloor < components.Aero.AeroComponent
    % UNDERBODYFLOOR Floor / diffuser / underbody aerodynamic model
    % Positioned near the CG. VERY highly sensitive to ride height (ground effect).
    % The floor is the most ride-height-dependent aero device on an FSAE car.
    %
    % Pitch behavior:
    %   - Nose UP → floor leading edge rises → air escapes under splitter → LESS downforce
    %   - Nose DOWN → floor leading edge drops → stronger underbody seal → MORE downforce
    %   BUT also: too low = stall / porpoising
    %   Net effect: negative pitch sensitivity (loses DF on nose-up)
    %
    % Height behavior:
    %   Downforce increases as floor gets closer to ground, up to a stalling limit.
    %   Modeled as an exponential relationship.

    properties
        stallHeight          = 0.015 % Below this height, floor stalls [m]
        heightExponent       = 0.6   % Controls ground-effect sensitivity curve
        floorLength          = 1.1   % Effective floor length used for pitch/rake clearances [m]
    end
    
    methods
        function obj = set.stallHeight(obj, value)
            obj.stallHeight = utils.positiveScalarOrDefault(value, 0.015);
        end

        function obj = set.heightExponent(obj, value)
            obj.heightExponent = utils.nonnegativeScalarOrDefault(value, 0.6);
        end

        function obj = set.floorLength(obj, value)
            obj.floorLength = utils.nonnegativeScalarOrDefault(value, 1.1);
        end

        function obj = UnderbodyFloor(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, stallHeight, heightExponent, floorLength)
            % UNDERBODYFLOOR Construct a floor/diffuser model
            %   UnderbodyFloor()
            %   UnderbodyFloor(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, stallHeight, heightExponent)
            %   UnderbodyFloor(..., floorLength)
            if nargin < 1 || isempty(xPosition)
                xPosition = 0;
            end
            if nargin < 2 || isempty(zPosition)
                zPosition = 0;
            end
            if nargin < 3 || isempty(ClA)
                ClA = 1.0;
            end
            if nargin < 4 || isempty(CdA)
                CdA = 0.5;
            end
            if nargin < 5 || isempty(pitchSensitivityClA)
                pitchSensitivityClA = 0;
            end
            obj@components.Aero.AeroComponent("Underbody Floor", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
            if nargin >= 6 && ~isempty(stallHeight)
                obj.stallHeight = utils.positiveScalarOrDefault( ...
                    stallHeight, obj.stallHeight);
            end
            if nargin >= 7 && ~isempty(heightExponent)
                obj.heightExponent = utils.nonnegativeScalarOrDefault( ...
                    heightExponent, obj.heightExponent);
            end
            if nargin >= 8 && ~isempty(floorLength)
                obj.floorLength = utils.nonnegativeScalarOrDefault( ...
                    floorLength, obj.floorLength);
            end
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: exponential ground-effect model. The floor is an
            % extended surface, so using one point at the CG misses rake: nose
            % up raises the leading edge and lowers the diffuser end. Use mean
            % clearance for the pressure-strength trend and minimum clearance
            % for choking/stall, which keeps the model tied to plausible floor
            % geometry without requiring a full CFD map.
            [meanClearance, minClearance] = obj.computeFloorClearances(vehicleState);
            
            if minClearance < obj.stallHeight
                % Stalling region: downforce drops rapidly
                stallFactor = (minClearance / obj.stallHeight)^2;
            else
                % Normal ground effect region
                stallFactor = 1.0;
            end
            
            referenceHeight = max(obj.zPosition, obj.stallHeight);
            heightFactor = stallFactor * (referenceHeight / meanClearance)^obj.heightExponent;
            
            % Clamp height factor to reasonable range
            heightFactor = max(0, min(heightFactor, 3.0));
            
            effectiveClA = obj.ClA * pitchFactor * heightFactor;
            effectiveClA = max(0, effectiveClA);
            F_downforce = obj.computeDownforceFromClA(vehicleState, effectiveClA);
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            % Floor drag is mostly from suction-induced pressure drag
            % Increases slightly with more downforce
            pitchFactor = 1 + 0.2 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            F_drag = obj.computeLongitudinalDragFromCdA(vehicleState, effectiveCdA);
        end

        function F_side = computeSideDrag(obj, vehicleState)
            % Reuse the same pressure-drag estimate as longitudinal drag, then
            % project it onto body-y so sideslip dissipates lateral kinetic
            % energy instead of being ignored by the aero model.
            pitchFactor = 1 + 0.2 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            F_side = obj.computeLateralDragFromCdA(vehicleState, effectiveCdA);
        end

        function [meanClearance, minClearance] = computeFloorClearances(obj, vehicleState)
            % COMPUTEFLOORCLEARANCES Estimate front/rear underbody heights.
            %
            % xPosition is the nominal center of the aerodynamic floor. Positive
            % pitch raises points forward of the CG and lowers points behind the
            % CG, matching the VehicleState convention used by the other aero
            % components.
            halfLength = max(obj.floorLength, 0) / 2;
            frontX = obj.xPosition + halfLength;
            rearX = obj.xPosition - halfLength;

            frontClearance = obj.zPosition + frontX * vehicleState.pitchAngle ...
                + vehicleState.rideHeight;
            rearClearance = obj.zPosition + rearX * vehicleState.pitchAngle ...
                + vehicleState.rideHeight;

            frontClearance = max(frontClearance, 0.005);
            rearClearance = max(rearClearance, 0.005);
            meanClearance = max(0.5 * (frontClearance + rearClearance), 0.005);
            minClearance = min(frontClearance, rearClearance);
        end
    end
end
