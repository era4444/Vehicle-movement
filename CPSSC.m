clear all

rand('seed',6)

%[X,Y] = MakeTrack(20,40,150,20);

%[X,Y] = MakeTrack(20,40,200,20);    %  ****
[X,Y] = MakeTrack(20,30,200,20);
%[X,Y] = MakeTrack(20,30,60,20);
%[X,Y] = MakeTrack(20,30,100,100);
%[X,Y] = MakeTrack(5,15,80,20);  % fix
[pathX, pathY] = PathToTurn(X, Y, [ X(1) Y(1) ], 1, 1, 0);

velocity = 1;   
turnError = 0.0;    % random error in radians
nextTurnIdx = 2;
curveEndT = 1;
carHeading = 0;
missedTurn = 0;
turnWasTruncated = 0;   % Turn was cut short because the car encountered another turn
%isCorrectingCourse = 0; % flag to prevent
v = [];

figure(1);
clf(1);
%fill(X,Y,'w','LineWidth',8,'edgecolor','y')
plot(X,Y,'y-','LineWidth',8)
hold on
plot(X,Y,'k+')  % plot black +'s at each turn
xlim([0 100])
axis auto

    %  store the point where the car merges with track after turn
X2 = zeros(1, size(X,2));
Y2 = zeros(1, size(Y,2));

    %  store the point where the car WILL MERGE with the NEXT track segment after turn
X3 = zeros(1, size(X,2));
Y3 = zeros(1, size(Y,2));

while nextTurnIdx < size(X,2)
    v = [ v sqrt(diff(pathY).^2 + diff(pathX).^2) ];
    for t = 1:size(pathX,2)
        C = [ pathX(t) pathY(t) ];  % car position

        [distanceToTurn, distanceToTrack] = DistanceToTurn(X,Y,C,carHeading,nextTurnIdx, t <= curveEndT)
        if missedTurn
            [d2, distanceToTrack2] = DistanceToTurn(X,Y,C,carHeading,nextTurnIdx+1, t <= curveEndT)
        end

        if missedTurn && ( distanceToTurn >= 10 && distanceToTrack >= 5 || d2 < 16)
                % passed the missed turn
            nextTurnIdx = nextTurnIdx + 1;
            missedTurn = 0;
            distanceToTurn = d2;
            distanceToTrack = distanceToTrack2;
        elseif distanceToTurn < 10 && ~missedTurn   % approaching turn
            if t <= curveEndT   % is car in a turn when it hits the next turn
                    % yes - re-calcalculate car heading where the car is at
                    % in the turn so TurnCar has the corect car heading
                carHeading = atan((pathY(t+1) - pathY(t)) / (pathX(t+1) - pathX(t)));
            end
            [curveX,curveY, xDot, yDot, carHeading, X2, Y2] = TurnCar(X, Y, X2, Y2, C, nextTurnIdx, velocity, distanceToTurn, turnError, carHeading, turnWasTruncated);
            if isempty(curveX) && isempty(curveY)
                [pathX,pathY] = PathToTurn(X,Y,C,velocity,nextTurnIdx+1, carHeading);   % calculate path to next turn
                curveEndT = 0;  % reset curve end time so program can turn car
                missedTurn = 1;
                break;
            else
                distanceOfCarToNextTurn = sqrt((X(nextTurnIdx+1) - C(1))^2 + (Y(nextTurnIdx+1) - C(2))^2);
                distanceToTurnFromEndOfCurve = sqrt((X(nextTurnIdx+1) - curveX(end))^2 + (Y(nextTurnIdx+1) - curveY(end))^2);
                distanceOfCarToEndOfCurve = sqrt((C(1) - curveX(end))^2 + (C(2) - curveY(end))^2);
                segmentLen = sqrt((X(nextTurnIdx+1) - X(nextTurnIdx))^2 + (Y(nextTurnIdx+1) - Y(nextTurnIdx))^2);
                if distanceToTurnFromEndOfCurve > distanceOfCarToNextTurn...    % <===  Way overshot turn
                    || ( distanceOfCarToEndOfCurve > distanceOfCarToNextTurn...
                        && segmentLen > distanceToTurnFromEndOfCurve)...
                        % Turn Stops too late ( see 'Turn Stops Too Late.png' )
                        % stop the turn before the next turn threshold
                    distanceOfPointToNextTurn = sqrt((X(nextTurnIdx+1) - curveX).^2 + (Y(nextTurnIdx+1) - curveY).^2);
                    pointsInTurnThresholdIdx = find(distanceOfPointToNextTurn < 15);
                    curveEndT = min(pointsInTurnThresholdIdx) - 1;
                    carHeading = atan((curveY(curveEndT+1) - curveY(curveEndT)) / (curveX(curveEndT+1) - curveX(curveEndT)));
                    curveX(curveEndT:end) = []; curveY(curveEndT:end) = []; % truncate curve 
                    turnWasTruncated = 1;
                else
                    turnWasTruncated = 0;
                end
                C = [ curveX(end) curveY(end) ];    % set new car position after turn
                [pathX,pathY] = PathToTurn(X,Y,C,velocity,nextTurnIdx, carHeading);   % calculate path to next turn

                if turnError ~= 0 && ~turnWasTruncated...  % <== Unless turn stops too late
                   && X2(nextTurnIdx) == 0 && Y2(nextTurnIdx) == 0     % <== and not already set
                        % car vector intercepts track other than turn junction
                        % we need to fool the program into thinking the turn
                        % junction is where the car intersected the track
                        % This makes the turn radius correct.  ( See Incorrect.png and
                        % Correct.png
                        % if there is no turnError, this is not needed

                        % So we store the curveX and curveY position, which is on
                        % the track in the 2nd element of the track and keep the
                        % original for distance calculations
                    X2(nextTurnIdx) = curveX(end);
                    Y2(nextTurnIdx) = curveY(end);
                end


                % combine curve path to new turn 
                curveX(end) = [];   curveY(end) = [];   % remove duplicate point that is in pathX(1) and pathY(1)
                pathX = [ curveX pathX ];
                pathY = [ curveY pathY];
                nextTurnIdx = nextTurnIdx + 1;
                curveEndT = length(curveX);   % turn end time so we don't try to make any more turns until this one is done
                break;
            end
        end

            %  check distance to track to see if we are veering too far
            % but make sure we're not in a turn
        if distanceToTrack > 5 && t > curveEndT
            % veering too far off course.  Car needs to correct its course
            [curveX,curveY, carHeading] = CorrectCourse(X, Y, C, carHeading, nextTurnIdx, velocity, 3*distanceToTrack, turnError / 10);
            
            C = [ curveX(end) curveY(end) ];    % set new car position after turn

            if missedTurn
                turnIdx = nextTurnIdx+1;
            else
                turnIdx = nextTurnIdx-1;
            end            
            [pathX,pathY] = PathToTurn(X,Y,C,velocity,turnIdx, carHeading);   % calculate path to next turn

                % Find point where car path intersects track to prevent
                    % correcting course again until the car crosses the track
            mCar = tan(carHeading);
            mTrack = (Y(nextTurnIdx) - Y(nextTurnIdx-1)) / (X(nextTurnIdx) - X(nextTurnIdx-1));
            intersectX = (mCar * C(1) -  mTrack * X(nextTurnIdx-1) - C(2) + Y(nextTurnIdx-1)) / (mCar - mTrack);
            intersectY = (-mTrack * C(2) + mCar*(mTrack*(C(1) - X(nextTurnIdx-1)) + Y(nextTurnIdx-1))) / (mCar - mTrack);
            distanceToTrackV = sqrt((intersectY - pathY).^2 + (intersectX - pathX).^2);
            intersectT = find(distanceToTrackV == min(distanceToTrackV));   % point in path closest to where car's path intersects track
            curveEndT = length(curveX) + intersectT;

            if turnError ~= 0
                X2(nextTurnIdx-1) = intersectX;
                Y2(nextTurnIdx-1) = intersectY;
            end


            % combine course correction path to new turn 
            curveX(end) = [];   curveY(end) = [];   % remove duplicate point that is in pathX(1) and pathY(1)
            pathX = [ curveX pathX ];
            pathY = [ curveY pathY];
            break;
        end
        MoveCar(pathX, pathY, t, velocity);
        PlotCarOrientation(pathX, pathY, t);
    end
end

    % plot the last leg
v = [ v sqrt(diff(pathY).^2 + diff(pathX).^2) ];    % don't forget to calculate velocities for the last leg
for t=1:size(pathX,2)
    MoveCar(pathX, pathY, t, velocity);
    PlotCarOrientation(pathX, pathY, t);
end

    % find times when velocity exceeded error
velocityErrorThreshold = .001;   % find where velocity exceeded maximum error
vErrorIdx = find(abs(v - 1) > velocityErrorThreshold);
vError = v(vErrorIdx);

figure(2);
clf(2);
plot(vError');
xlabel('Time');
ylabel('Velocity');
title(['Points where velocity exceeded ' num2str(velocityErrorThreshold)]);
