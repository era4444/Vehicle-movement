function [curveX,curveY, carHeading] = CorrectCourse(X, Y, C, carHeading, jctnIdx, thetaPrime, r, turnError)  
    A = [ X(jctnIdx-1) Y(jctnIdx-1) ];
    B = [ X(jctnIdx) Y(jctnIdx) ];
    AB = B - A;    
    AC = C - A;

    alpha = acos(dot(AB,AC) / (sqrt(AB(1)^2 + AB(2)^2) * sqrt(AC(1)^2 + AC(2)^2)));
    thetaAC = -atan(AC(2) / AC(1));
    a1 = -acot(AC(2) / AC(1));

    circleXBelow = C(1) + r^2 * (C(2) - A(2))^2 / sqrt(r^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2) * (C(2) - A(2))^2);
    circleYBelow = (C(1) * (-sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2)-A(2))^2)))...
        + A(1) * sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2)) ...
    + C(2) * (C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
    / ((C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2));

    circleXAbove = C(1) - r^2 * (C(2) - A(2))^2 / sqrt(r^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2) * (C(2) - A(2))^2);
    circleYAbove = (C(1) * sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2)-A(2))^2))...
        - A(1) * sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
    + C(2) * (C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
    / ((C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2));

    dBelow = sqrt((B(1) - circleXBelow)^2 + (B(2) - circleYBelow)^2);
    dAbove = sqrt((B(1) - circleXAbove)^2 + (B(2) - circleYAbove)^2);

    if dBelow < dAbove
            % Car is Below the track
        arcX = r^2 * (C(2) - A(2))^2 / sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            - r * cos(2 * acos( ((C(1)-A(1))*(B(1) - A(1)) + (C(2)-A(2))*(B(2)-A(2)))...
                / ( sqrt((C(1)-A(1))^2 + (C(2)-A(2))^2) * sqrt((A(1)-B(1))^2 + (A(2)-B(2))^2)))...
            + atan((C(1)-A(1))/(C(2)-A(2)))) + C(1);

        arcY = (C(1) * (-sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2)))...
            + A(1) * sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            + C(2) * (C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            / ((C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            + r * sin(2*acos( ((C(1) - A(1)) * (B(1) - A(1)) + (C(2) - A(2)) * (B(2) - A(2))) / ( sqrt( (C(1) - A(1))^2 + (C(2) - A(2))^2) * sqrt((A(1) - B(1))^2 + (A(2) - B(2))^2)))...
            + atan((C(1) - A(1)) / (C(2) - A(2))));
        
            a2 = atan(cot(thetaAC)) - 2 * alpha;
            a2 = a2 - turnError + 2*turnError * rand(1);    % random error
            
            arcLen = abs(a2 - a1) * r;
            numPoints = arcLen / thetaPrime;
            thetaPrime = (a2 - a1) / numPoints;

            th = a1 + pi:thetaPrime:a2 + pi;
            circleCenter = [ circleXBelow circleYBelow ];
    else
            % Car is Above the track
        arcX =- r^2 * (C(2) - A(2))^2 / sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
        + r * cos(2 * acos( ((C(1)-A(1))*(B(1) - A(1)) + (C(2)-A(2))*(B(2)-A(2)))...
            / ( sqrt((C(1)-A(1))^2 + (C(2)-A(2))^2) * sqrt((A(1)-B(1))^2 + (A(2)-B(2))^2)))...
        - atan((C(1)-A(1))/(C(2)-A(2)))) + C(1);

        arcY = (C(1) * sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            - A(1) * sqrt(r^2 * (C(2) - A(2))^2 * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            + C(2) * (C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            / ((C(2) - A(2)) * ((C(1) - A(1))^2 + (C(2) - A(2))^2))...
            + r * sin(2*acos( ((C(1) - A(1)) * (B(1) - A(1)) + (C(2) - A(2)) * (B(2) - A(2))) / ( sqrt( (C(1) - A(1))^2 + (C(2) - A(2))^2) * sqrt((A(1) - B(1))^2 + (A(2) - B(2))^2)))...
            - atan((C(1) - A(1)) / (C(2) - A(2))));

            a2 = atan(cot(thetaAC)) + 2 * alpha;
            a2 = a2 - turnError + 2*turnError * rand(1);    % random error

            arcLen = abs(a2 - a1) * r;
            numPoints = arcLen / thetaPrime;
            thetaPrime = (a2 - a1) / numPoints;

            th = a1:thetaPrime:a2 ;
            circleCenter = [ circleXAbove circleYAbove ];
    end
    curveX = circleCenter(1) + r*cos(th);
    curveY = circleCenter(2) + r*sin(th);

    if turnError 
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
