function [ distance1, angle1 ] = DistanceCalculation( pathX,pathY)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

for i=1:length(pathX)-1
    
    X=pathX(i+1)-pathX(i);
    Y=pathY(i+1)-pathY(i);
    distance1(i)=sqrt(X^2+Y^2);
    angle_rad(i)=atan(Y/X);
    angle1(i)=radtodeg(angle_rad(i));

end

