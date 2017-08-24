function [turnDistance, trackDistance] = DistanceToTurn(X,Y,C,carHeading,jctnIdx, carIsTurning)
    x2 = X(jctnIdx);
    y2 = Y(jctnIdx);
    
    a = C(1);
    b = C(2);
    
    if jctnIdx == length(X)
        x1 = X(1);
        y1 = Y(1);
    else
        x1 = X(jctnIdx-1);
        y1 = Y(jctnIdx-1);
    end

    turnDistance = sqrt((a-x2)^2 + (b-y2)^2);
    if carHeading ~= 0 && ~carIsTurning && jctnIdx < length(X)   % <== no next track segment intercept on last piece of track
            % also check distance to where car path will intersect next track
            % segment to prevent missing the turn
        B = [ X(jctnIdx+1) Y(jctnIdx+1) ];
        A = [ X(jctnIdx) Y(jctnIdx) ];
        AB = B - A;
        mTrack = AB(2) / AB(1);  % slope of next track segment
        mCar = tan(carHeading); % slope of car's path

        interceptA(1) =  C(1)*mCar - B(1)*mTrack - C(2) + B(2);
        interceptA(2) = -mTrack*C(2) + mCar*(mTrack*(C(1)-B(1))+B(2));
        interceptA = interceptA / ( mCar - mTrack );
        
        distanceOfInterceptToThisTurn = sqrt((interceptA(1)-x1)^2 + (interceptA(2)-y1)^2);
        distanceOfInterceptToNextTurn = sqrt((interceptA(1)-x2)^2 + (interceptA(2)-y2)^2);
        
        if distanceOfInterceptToNextTurn < distanceOfInterceptToThisTurn  % <== if path of car does not intercept next track segment behind the car
                % return distance to intercept or distance to turn
                % whichever is smaller
            distanceOfCarToIntercept = sqrt((interceptA(1) - C(1))^2 + (interceptA(2) - C(2))^2);
            turnDistance = min(turnDistance, distanceOfCarToIntercept);
        end
            % else ignore because the intercept is behind the car
    end
    
    theta = acos(((a-x1) * (-x1+x2) + (b-y1) * (y2-y1)) / (sqrt((a-x1)^2 + (b-y1)^2) * sqrt((x2-x1)^2 + (y2-y1)^2)));
%    if theta > pi / 2 && jctnIdx > 2   % closer to previous segment of track

    trackDistance = sqrt((a-x1)^2 + (b-x2)^2) * sqrt(1 - ((a-x1) * (x2-x1) + (b-y1) * (y2-y1))^2 / (((a-x1)^2 + (b-y1)^2) * ((x2-x1)^2 + (y2-y1)^2)));
    if jctnIdx > 2
        x1 = X(jctnIdx - 2);        y1 = Y(jctnIdx - 2);
        x2 = X(jctnIdx - 1);        y2 = Y(jctnIdx - 1);
        thisSegmentDistance = sqrt((a-x1)^2 + (b-x2)^2) * sqrt(1 - ((a-x1) * (x2-x1) + (b-y1) * (y2-y1))^2 / (((a-x1)^2 + (b-y1)^2) * ((x2-x1)^2 + (y2-y1)^2)));
    
        % return the closest distance from either segment ( in case we're
        % turning a corner
        trackDistance = min(thisSegmentDistance, trackDistance);
    end
end
