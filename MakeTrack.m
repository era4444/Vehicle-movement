function [X,Y,len] = MakeTrack(minStep, stepIncrement, minRange, rangeIncrement)

%      xTop = -minRange:minStep+stepIncrement*rand():minRange;
%      yTop = minRange+rangeIncrement*rand(1,size(xTop,2));

% 
%     yRight = minRange:-(minStep+stepIncrement*rand()):-minRange; yRight(1) = [];
%     xRight = minRange+rangeIncrement*rand(1,size(yRight,2));
% 
%     xBottom = minRange:-(minStep+stepIncrement*rand()):-minRange;    xBottom(1) = [];
%     yBottom = -minRange-rangeIncrement*rand(1,size(xBottom,2));
    
%     yLeft = -minRange:minStep+stepIncrement*rand():minRange;    yLeft(1) = [];
%     xLeft = -minRange-rangeIncrement*rand(1,size(yLeft,2));

     Y = 0:minStep+stepIncrement*rand():2*minRange;    Y(1) = [];
     X = rangeIncrement*rand(1,size(Y,2));

%     X = [ xTop minRange+rangeIncrement/2 xRight (minRange+rangeIncrement/2)  xBottom -(minRange+rangeIncrement/2) xLeft ];
%     Y = [ yTop (minRange+rangeIncrement/2) yRight -(minRange+rangeIncrement/2) yBottom -(minRange+rangeIncrement/2) yLeft ];
    len = length(X);    % # of turns in track
end
