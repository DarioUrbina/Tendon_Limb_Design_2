% clear all
% clc


% q1=[0, 15];
% q2=[-.1 ,8];
% foot=[1,0];
% 
% x1=[q1(1) q2(1)];
% y1=[q1(2) q2(2)];
% 
% x2=[q2(1) foot(1)];
% y2=[q2(2) foot(2)];

x1=[0 -2.57];
y1=[15 7.42];

x2=[-2.57 0.4];
y2=[7.42 0];


[deg1,deg2]=MeasureAngle (x1, y1, x2, y2)
