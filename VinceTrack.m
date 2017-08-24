clear all

rand('seed',3)

%[X,Y] = MakeTrack(20,30,200,20);
%[X,Y] = MakeTrack(20,30,60,20);
[X,Y] = MakeTrack(20,30,100,100);
%[X,Y] = MakeTrack(5,15,80,20);
[pathX, pathY] = PathToTurn(X, Y, [ X(1) Y(1) ], 1, 1, 0);

velocity = 1;   
turnError = 0.1;    % random error in radians
nextTurnIdx = 2;
curveEndT = 1;
v = [];

figure(1);
clf(1);
%fill(X,Y,'w','LineWidth',8,'edgecolor','y')
plot(X,Y,'y-','LineWidth',8)
hold on
plot(X,Y,'k+')  % plot black +'s at each turn
xlim([0 100])
axis auto


while nextTurnIdx < size(X,2)
    v = [ v sqrt(diff(pathY).^2 + diff(pathX).^2) ];
    for t = 1:size(pathX,2)
        C = [ pathX(t) pathY(t) ];  % car position

        [d, distanceToTrack] = DistanceToTurn(X,Y,C,nextTurnIdx)

            %  check distance to track to see if we are veering too far
            % but make sure we're not in a turn
        if distanceToTrack > 5 && t > curveEndT
            % veering too far off course
            disp('Change Course');
        end

        if d < 10    % approaching turn
            [curveX,curveY, xDot, yDot, newCarHeading] = TurnCar(X, Y, C, nextTurnIdx, velocity, d, turnError);
            C = [ curveX(end) curveY(end) ];    % set new car position after turn
            [pathX,pathY] = PathToTurn(X,Y,C,velocity,nextTurnIdx, newCarHeading);   % calculate path to next turn

            % combine curve path to new turn 
            curveX(end) = [];   curveY(end) = [];   % remove duplicate point
            pathX = [ curveX pathX ];
            pathY = [ curveY pathY];
            nextTurnIdx = nextTurnIdx + 1;
            curveEndT = size(curveX,2);   %
            break;
        end


        plot(pathX(t),pathY(t),'r.');
        drawnow;
        pause(.03);
    end
end

    % plot the last leg
for t=1:size(pathX,2)
    plot(pathX(t),pathY(t),'r.');
    drawnow;
    pause(.03);
end


