
    % calculates the turn path and returns the path in curveX and curveY
    % also returns the angular displacement of the end of the turn in a2e
    % cot(a2e) is the slope of the car's new trajectory
function [curveX,curveY, xDot, yDot, carHeading, X2, Y2] = TurnCar(X, Y, X2, Y2, C, jctnIdx, thetaPrime, h, turnError, saveCarHeading, turnWasTruncated)    
    % store track segment endpoints for clockwise direction for calculating angle
        % between car heading and track
    A = [ X(jctnIdx) Y(jctnIdx) ];
    
    if jctnIdx == size(X,2)
        B = [ X(1) Y(1) ];
    else
        B = [ X(jctnIdx+1) Y(jctnIdx+1) ];
    end    
    AB = B - A;    
    AC = C - A;

        % adjust turn if car is off course
    if turnError ~= 0 && jctnIdx > 2        
            % find where car intersected this segment track
        thisA = [ X(jctnIdx-1) Y(jctnIdx-1) ];
        thisB = A;        
        thisAB = thisB - thisA; 

        mCar = tan(saveCarHeading);
        mThisTrackSegment = thisAB(2) / thisAB(1);
        
        A2(1) = mCar * C(1) - mThisTrackSegment * thisA(1) - C(2) + thisA(2);
        A2(2) = -mThisTrackSegment * C(2) + mCar*(mThisTrackSegment * (C(1) - thisA(1)) + thisA(2));
        A2 = A2 / ( mCar - mThisTrackSegment ); % place where car cross this segment of track
        
        % calculate the place where the car's path intersects with the next track segment
        % This will be the turn junction if there is no turn error
            
        A2C = C - A2;
        mNextTrackSegment = AB(2) / AB(1);  % slope of next track segment

        interceptA(1) = B(2) - A2(2) + A2(1)*mCar - B(1)*mNextTrackSegment;
        interceptA(2) = B(2)*mCar - (A2(2) + (B(1) - A2(1))*mCar)*mNextTrackSegment;
        interceptA = interceptA / ( mCar - mNextTrackSegment );
        X2(jctnIdx) = interceptA(1); Y2(jctnIdx) = interceptA(2);
        AB = B - interceptA;    
        AC = C - interceptA;

            % Make sure the point where the next segment intersects with
            % the path of the car is not behind the car

%             distanceOfInterceptToTurn = sqrt((interceptA(2) - A(2))^2 + (interceptA(1) - A(1))^2);
%         distanceOfCarToTurn = sqrt((C(2) - A(2))^2 + (C(1) - A(1))^2);
        distanceOfCarToNextTurn = sqrt((C(2) - B(2))^2 + (C(1) - B(1))^2);
        distanceOfInterceptToNextTurn = sqrt((B(2) - interceptA(2))^2 + (B(1) - interceptA(1))^2);

        distanceOfCarToA2 = sqrt((C(2) - A2(2))^2 + (C(1) - A2(1))^2);
        distanceOfInterceptToA2 = sqrt((A2(1) - interceptA(1))^2 + (A2(2) - interceptA(2))^2);
        distanceOfCarToPreviousTurn = sqrt((thisA(1) - C(1))^2 + (thisA(2) - C(2))^2);
        distanceOfInterceptToPreviousTurn = sqrt((thisA(1) - interceptA(1))^2 + (thisA(2) - interceptA(2))^2);
%            segmentLen = sqrt((B(2) - A(2))^2 + (B(1) - A(1))^2);

        if distanceOfCarToA2 < distanceOfInterceptToA2 && distanceOfCarToNextTurn > distanceOfInterceptToNextTurn...
            || distanceOfInterceptToPreviousTurn > distanceOfCarToPreviousTurn...       % intercept is in front of car
            || turnWasTruncated    % because this turn was hit before car finished the previous curve, so we know there is a turn.
            h = sqrt((interceptA(2) - C(2))^2 + (interceptA(1) - C(1))^2);    % recalculate distance to new turn junction
            A = interceptA;
        else
                % next track segment intersected car's path behind the car
            curveX = [];    curveY = [];
            xDot = [];  yDot = [];
            carHeading = saveCarHeading;
            return
        end
    end

    magAB = sqrt(AB(1)^2 + AB(2)^2);
    magAC = sqrt(AC(1)^2 + AC(2)^2);

        % Angle of lines before / after to know if turning right or left
    th1 = atan2(AB(2), AB(1)); % atan2 - angles in all 4 quadrants
    th2 = atan2(AC(2), AC(1));
    
    theta = acos(dot(AB,AC) / ( magAB * magAC ));
    alpha = pi - theta;
    r = h * cot(alpha / 2);
        
    P = [ A(1) + h / magAB * AB(1) A(2) + h / magAB * AB(2) ];
    endTurnAngle = atan(-AB(1) / AB(2));

    if th1 - th2 > pi % turn left
        circleCenter = [ P(1) - r * cos(endTurnAngle) P(2) - r * sin(endTurnAngle) ];
        a1 = atan(((C(2)) - circleCenter(2)) / (C(1) - circleCenter(1)));
        a2 = atan((P(2) - circleCenter(2)) / (P(1) - circleCenter(1)));
        
        % random error experiment
        oldA2 = a2; % test to make sure random error doesn't
        a2 = a2 - turnError + 2*turnError * rand(1);
        if sign(a2 - a1) ~= sign(oldA2 - a1)
                % very shallow turn skip the turn error
                % see "Random Error Causes Car to Reverse.png"
            a2 = oldA2;
        end

        arcLen = abs(a2 - a1 ) * r;   % theta * r
        numPoints = arcLen / thetaPrime;
        thetaPrime = (a2 - a1) / numPoints;
        
        th = a1:thetaPrime:a2;
    else    % turn right
            % find center of turn
        circleCenter = [ P(1) + r * cos(endTurnAngle) P(2) + r * sin(endTurnAngle) ];
        a1 = atan((circleCenter(2) - C(2)) / (circleCenter(1) - C(1)));
        a2 = atan((circleCenter(2) - P(2)) / (circleCenter(1) - P(1)));   

        % random error experiment
        oldA2 = a2; % test to make sure random error doesn't
        a2 = a2 - turnError + 2*turnError * rand(1);
        if sign(a2 - a1) ~= sign(oldA2 - a1)
                % very shallow turn skip the turn error
                % see "Random Error Causes Car to Reverse.png"
            a2 = oldA2;
        end

        arcLen = abs(a2 - a1) * r;
        numPoints = arcLen / thetaPrime;
        thetaPrime = (a2 - a1) / numPoints;
        
        th = a1 + pi:thetaPrime:a2+pi;
    end
    curveX = circleCenter(1) + r*cos(th);
    curveY = circleCenter(2) + r*sin(th);
    
    if turnError 
%         m = (curveY(end) - curveY(end-1)) / (curveX(end) - curveX(end-1));
        m = (curveY(end) - circleCenter(2)) / (curveX(end) - circleCenter(1));
        carHeading = atan(-1/m);    % cotangent
        if carHeading < 0   
            carHeading = carHeading + pi; % never heading down the screen
        end
    else
        carHeading = 0;
    end

        % velocity ( derivative )
    xDot = -r * sin(th) * thetaPrime;
    yDot = r * cos(th) * thetaPrime;
end
