classdef CorrelationTrackAlignment
    % CORRELATIONTRACKALIGNMENT Aligns imported MoTeC laps to model tracks.

    methods (Static)
        function [station, errorDeg, sampleCount] = estimateStartStationFromCourse( ...
                track, profile, alignmentDistanceM, alignmentStepM)
            distance = profile.distance(:);
            course = profile.gpsCourse(:);
            maxDistance = min(alignmentDistanceM, max(distance(isfinite(distance))));
            keep = isfinite(distance) & isfinite(course) & ...
                distance >= 0 & distance <= maxDistance;

            if nnz(keep) < 3
                error('lts_correlation_CorrelationTrackAlignment:InsufficientCourse', ...
                    'At least three finite gps_course_rad samples are required for automatic alignment.');
            end

            queryDistance = distance(keep);
            queryDistance = queryDistance - queryDistance(1);
            queryCourse = course(keep);
            sampleCount = numel(queryDistance);
            loggedCurvature = lts.correlation.CorrelationTrackAlignment.loggedCurvatureSignals( ...
                profile, keep);

            [trackS, trackHeading, trackLength] = ...
                lts.correlation.CorrelationTrackAlignment.trackHeadingSamples(track);
            [trackCurvS, trackCurvature, ~] = ...
                lts.correlation.CorrelationTrackAlignment.trackCurvatureSamples(track);
            candidates = (0:alignmentStepM:trackLength).';
            if numel(candidates) > 1 && candidates(end) >= trackLength
                candidates(end) = [];
            end

            scores = inf(size(candidates));
            meanErrors = inf(size(candidates));
            firstErrors = inf(size(candidates));
            initialHeadingTolerance = 30 * pi / 180;
            initialHeadingWeight = 10;
            headingWindowWeight = 0.02;
            curvatureWeight = 20;
            for i = 1:numel(candidates)
                queryStation = candidates(i) + queryDistance;
                modelHeading = lts.correlation.CorrelationTrackAlignment.headingAtStation( ...
                    trackS, trackHeading, trackLength, ...
                    queryStation);
                err = lts.correlation.CorrelationTrackAlignment.angularDiff(modelHeading, queryCourse);
                finiteErr = abs(err(isfinite(err)));
                if ~isempty(finiteErr)
                    meanErrors(i) = mean(finiteErr);
                    firstErrors(i) = abs(err(1));
                    if firstErrors(i) <= initialHeadingTolerance
                        curvatureCost = lts.correlation.CorrelationTrackAlignment.curvatureCost( ...
                            trackCurvS, trackCurvature, trackLength, ...
                            queryStation, loggedCurvature);
                        if isfinite(curvatureCost)
                            scores(i) = initialHeadingWeight * firstErrors(i) + ...
                                headingWindowWeight * meanErrors(i) + ...
                                curvatureWeight * curvatureCost;
                        else
                            scores(i) = meanErrors(i) + initialHeadingWeight * firstErrors(i);
                        end
                    end
                end
            end

            if all(~isfinite(scores))
                warning('lts_correlation_CorrelationTrackAlignment:NoInitialHeadingMatch', ...
                    ['No start-station candidate matched the initial GPS course ' ...
                     'within 30 deg. Falling back to weighted heading-window score.']);
                scores = meanErrors + initialHeadingWeight * firstErrors;
            end

            [score, idx] = min(scores);
            station = candidates(idx);
            if isfinite(meanErrors(idx))
                errorDeg = meanErrors(idx) * 180 / pi;
            else
                errorDeg = score * 180 / pi;
            end
        end

        function trackOut = rebaseTrack(track, startStationM)
            points = track.getTrackPoints();
            closed = lts.correlation.CorrelationTrackAlignment.isClosedTrack(track, points);
            if ~closed
                if abs(startStationM) <= 1e-9
                    trackOut = track;
                    return;
                end
                error('lts_correlation_CorrelationTrackAlignment:OpenTrackRebase', ...
                    'Only closed tracks can be rebased to a nonzero start station.');
            end

            trackLength = track.getTotalLength();
            startStationM = lts.correlation.CorrelationTrackAlignment.wrapStation(startStationM, trackLength);
            if abs(startStationM) <= 1e-9
                trackOut = track;
                return;
            end

            s = lts.components.Track.cumulativeArcLength(points, true);
            p = [points; points(1, :)];
            idx = find(s <= startStationM, 1, 'last');
            idx = min(idx, numel(s) - 1);
            if isempty(idx)
                idx = 1;
            end

            segmentLength = max(s(idx + 1) - s(idx), eps);
            t = (startStationM - s(idx)) / segmentLength;
            startPoint = (1 - t) * p(idx, :) + t * p(idx + 1, :);

            if t <= 1e-9
                rebasedPoints = [points(idx:end, :); points(1:idx-1, :)];
            else
                rebasedPoints = [startPoint; points(idx+1:end, :); points(1:idx, :)];
            end

            mu = track.getSurfaceFriction();
            if isscalar(mu)
                rebasedMu = mu;
            else
                mu = mu(:);
                muStart = (1 - t) * mu(idx) + t * mu(mod(idx, numel(mu)) + 1);
                if t <= 1e-9
                    rebasedMu = [mu(idx:end); mu(1:idx-1)];
                else
                    rebasedMu = [muStart; mu(idx+1:end); mu(1:idx)];
                end
            end

            % Forward the per-waypoint corridor through the rebase. Rebasing is
            % a pure rotation of waypoint order (not a reversal), so the left
            % and right sides keep their identity -- only the index order
            % changes, with the same rotation applied to mu above. A scalar
            % width track stays scalar.
            widthArgs = {};
            if ismethod(track, 'getTrackSideWidths')
                [leftWidth, rightWidth] = track.getTrackSideWidths();
                if numel(leftWidth) == size(points, 1) && numel(rightWidth) == size(points, 1)
                    leftWidth = leftWidth(:);
                    rightWidth = rightWidth(:);
                    leftStart = (1 - t) * leftWidth(idx) + ...
                        t * leftWidth(mod(idx, numel(leftWidth)) + 1);
                    rightStart = (1 - t) * rightWidth(idx) + ...
                        t * rightWidth(mod(idx, numel(rightWidth)) + 1);
                    if t <= 1e-9
                        rebasedLeft = [leftWidth(idx:end); leftWidth(1:idx-1)];
                        rebasedRight = [rightWidth(idx:end); rightWidth(1:idx-1)];
                    else
                        rebasedLeft = [leftStart; leftWidth(idx+1:end); leftWidth(1:idx)];
                        rebasedRight = [rightStart; rightWidth(idx+1:end); rightWidth(1:idx)];
                    end
                    widthArgs = {'LeftWidth', rebasedLeft, 'RightWidth', rebasedRight};
                end
            end
            if isempty(widthArgs)
                widthArgs = {'Width', track.getTrackWidth()};
            end

            metadata = struct();
            if isprop(track, 'Metadata') && isstruct(track.Metadata)
                metadata = track.Metadata;
            end
            metadata.correlation_alignment_start_station_m = startStationM;
            metadata.correlation_alignment_source = 'lts.correlation.CorrelationTrackAlignment';

            name = 'CorrelationTrack';
            if isprop(track, 'Name') && ~isempty(track.Name)
                name = char(track.Name);
            end

            trackOut = lts.components.WaypointTrack(rebasedPoints, ...
                widthArgs{:}, ...
                'Mu', rebasedMu, ...
                'Closed', true, ...
                'Name', sprintf('%s rebased %.1fm', name, startStationM), ...
                'Metadata', metadata);
        end

        function errorDeg = initialHeadingErrorDeg(track, profile)
            errorDeg = NaN;
            if ~profile.hasGpsCourse()
                return;
            end
            heading = track.getHeading();
            if isempty(heading)
                return;
            end
            course = profile.gpsCourse(isfinite(profile.gpsCourse));
            if isempty(course)
                return;
            end
            errorRad = lts.correlation.CorrelationTrackAlignment.angularDiff(heading(1), course(1));
            errorDeg = abs(errorRad) * 180 / pi;
        end

        function [trackS, trackHeading, trackLength] = trackHeadingSamples(track)
            points = track.getTrackPoints();
            closed = lts.correlation.CorrelationTrackAlignment.isClosedTrack(track, points);
            trackS = lts.components.Track.cumulativeArcLength(points, closed);
            trackHeading = track.getHeading();
            if closed
                trackHeading = [trackHeading(:); trackHeading(1)];
            else
                trackHeading = trackHeading(:);
            end
            trackLength = trackS(end);
        end

        function [trackS, trackCurvature, trackLength] = trackCurvatureSamples(track)
            points = track.getTrackPoints();
            closed = lts.correlation.CorrelationTrackAlignment.isClosedTrack(track, points);
            trackS = lts.components.Track.cumulativeArcLength(points, closed);
            trackCurvature = track.getCurvature();
            if closed
                trackCurvature = [trackCurvature(:); trackCurvature(1)];
            else
                trackCurvature = trackCurvature(:);
            end
            trackLength = trackS(end);
        end

        function heading = headingAtStation(trackS, trackHeading, trackLength, queryS)
            queryS = mod(queryS, trackLength);
            cosH = interp1(trackS, cos(trackHeading), queryS, 'linear');
            sinH = interp1(trackS, sin(trackHeading), queryS, 'linear');
            heading = atan2(sinH, cosH);
        end

        function loggedCurvature = loggedCurvatureSignals(profile, keep)
            loggedCurvature = {};
            speed = profile.speed(keep);
            speed = max(abs(speed), 0.5);
            if ~isempty(profile.yawRate)
                yawRate = profile.yawRate(keep);
                yawCurvature = yawRate ./ speed;
                if any(isfinite(yawCurvature))
                    loggedCurvature{end+1} = yawCurvature(:); %#ok<AGROW>
                end
            end
            if profile.hasLatAccel()
                latAccel = profile.latAccelG(keep) * 9.80665;
                latCurvature = latAccel ./ max(speed.^2, 0.25);
                if any(isfinite(latCurvature))
                    loggedCurvature{end+1} = latCurvature(:); %#ok<AGROW>
                end
            end
        end

        function cost = curvatureCost(trackS, trackCurvature, trackLength, queryS, loggedCurvature)
            cost = NaN;
            if isempty(loggedCurvature)
                return;
            end
            queryS = mod(queryS, trackLength);
            modelCurvature = interp1(trackS, trackCurvature, queryS, 'linear');
            costs = NaN(numel(loggedCurvature), 1);
            for i = 1:numel(loggedCurvature)
                logged = loggedCurvature{i};
                eSame = lts.correlation.CorrelationTrackAlignment.meanFinite(abs(modelCurvature(:) - logged(:)));
                eFlipped = lts.correlation.CorrelationTrackAlignment.meanFinite(abs(modelCurvature(:) + logged(:)));
                costs(i) = min(eSame, eFlipped);
            end
            cost = lts.correlation.CorrelationTrackAlignment.meanFinite(costs);
        end

        function value = meanFinite(values)
            values = values(isfinite(values));
            if isempty(values)
                value = NaN;
            else
                value = mean(values);
            end
        end

        function diff = angularDiff(a, b)
            diff = atan2(sin(a - b), cos(a - b));
        end

        function station = wrapStation(station, trackLength)
            if ~isfinite(station)
                error('lts_correlation_CorrelationTrackAlignment:InvalidStation', ...
                    'Start station must be finite.');
            end
            trackLength = max(trackLength, eps);
            station = mod(station, trackLength);
            if abs(station - trackLength) < 1e-9
                station = 0;
            end
        end

        function closed = isClosedTrack(track, points)
            if ismethod(track, 'isClosedLoop')
                closed = track.isClosedLoop();
            elseif isprop(track, 'closedLoop')
                closed = track.closedLoop;
            elseif isprop(track, 'Closed')
                closed = track.Closed;
            else
                closed = size(points, 1) > 2 && ...
                    norm(points(1, :) - points(end, :)) <= 0.05;
            end
            closed = logical(closed);
        end
    end
end
