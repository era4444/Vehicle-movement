
    %   calculate vector tracjectory to the track
function [x,y] = PathToTurn(trackX, trackY, car, stepSize, idx, alpha)

        % find the closest turn junction
%    idx = ClosestTurn(trackX,trackY,car);
    
    endX = trackX(idx+1);
    if alpha == 0    % no error - car is aligned perfectly with track
        endY = trackY(idx+1);   % car path ends at turn
    else    % car is not lined perfectly on track
            % use slope of line to calculate where path of car ends
        m = tan(alpha);
        endY = m * (endX - car(1)) + car(2);    % equation of a line
        if endY < trackY(idx+1) % make sure have enough points to the next corner
            endY = trackY(idx+1);
            endX = (endY - car(2)) / m + car(1);
        end
%         trackX(idx+1) = endX;
%         trackY(idx+1) = endY;
    end
    
        % find path length for calculating number 
    len = sqrt((endY - car(2))^2 + (endX - car(1))^2);
    numPoints = len / stepSize; % points required in segement for constant velocity

        % calculate steps based on the maximum of x or y component
    if abs(endX) > abs(endY)
        stepSize = (endX - car(1)) / numPoints;
        if car(1) > endX
            stepSize = -stepSize;
        end
        x = car(1):stepSize:endX;
        
        yStep = (endY - car(2)) / (size(x,2)-1);
        y = car(2):yStep:endY;
    else
        stepSize = (endY - car(2)) / numPoints;
        if car(2) > endY
            stepSize = -stepSize;
        end
        y = car(2):stepSize:endY;
        
        xStep = (endX - car(1)) / (size(y,2)-1);
        x = car(1):xStep:endX;
    end
end
