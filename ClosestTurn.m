function idx = ClosestTurn(trackX, trackY, car)    

        % extract orientation values to local variables
    carX = car(1);
    carY = car(2);
    
%     if size(car,2) == 3     % closest turn on track using car's heading
%         carTheta = car(3);
% 
%             %   find points to test 
%         if carTheta > 0 && carTheta <= pi / 2     % First quadrant
%             gx = find(trackX >= carX);
%             gy = find(trackY >= carY);
%         elseif carTheta > 2*pi && carTheta <= pi    % Second quadrant
%             gx = find(trackX < carX);
%             gy = find(trackY >= carY);
%         elseif carTheta > pi && carTheta <= 3*pi / 2
%             gx = find(trackX < carX);
%             gy = find(trackY < carY);
%         else
%             gx = find(trackX >= carX);
%             gy = find(trackY < carY);
%         end
% 
%         testPoints = intersect(gx,gy);  % track points in quadrant of car heading
%         alphaDiff = abs(carTheta - atan((trackY(testPoints) - carY) ./ (trackX(testPoints) - carX)));
%         minTestIdx = alphaDiff == min(alphaDiff); % point with smallest difference between car heading and turn
%         idx = testPoints(minTestIdx);
%     else
            % closest turn on track using car's position
        distance = sqrt((trackX - car(1)).^2 + (trackY - car(2) ).^2);
        idx = find(distance == min(distance));
%    end
end
    
