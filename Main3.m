%% Main 
% Code to add leg dimensions, both segments the same size. 
% code to add motor parameters, forces and the scaling of the feasible force set to fit into figure
% Locations of knee and end point are calulated. (q1 position and floor location)
% CALLED FUNCTIONS: 
% This file calls the function MeasureAngle.m to calculate joint angles
%       Calculated angles have been tested, they are correctly obtained
% it also calls function Tendon_Limb_Design.m to get the feasible force set
%       Still need to determine if gotten feasible force set is good 


%% Parameters
%Physical measures
l1=18;                   %Leg segmanet measurement, both segments are the same                   
q1_Position=[0 15];     %Position of q1 (hip)        
firstTouchPoint=5.7;    %Useful as a initial conditions for calculations and 
%Motors
Motor_Force=49*.80;   %In a 24 24 scenario. The motors don't go to stall torque case, which would be 49 Nw
force_figure_scale=.1; %Scale to fit leg trajectory figure with feasible force sets figure 
%Figures
xlimit1=[-8 8];        
ylimit1=[-5 17]; 

%Aditional
angleCount=1;
touch_number=1;
%%
figure
for i=1:3.6:45
    angleCount;
    x(3) = firstTouchPoint-i; %This determines the end point touching points
    mid_Hip_x=(q1_Position(1)-x(3))/2; %half distance (x component) from q1 to end point touching points
    mid_Hip_y=(q1_Position(2))/2;       %half distance (y component) from q1 to end point touching points
    
    if (x(3)<0)
        foot_q1_dist=sqrt(((q1_Position(1)+x(3))^2)+(q1_Position(2))^2); %half distance (diagonal (hypothenus)) from q1 to end point touching points
    else
        foot_q1_dist=sqrt(((q1_Position(1)-x(3))^2)+(q1_Position(2))^2);   
    end
    
    
    if (foot_q1_dist<(2*l1))
        
        c=sqrt((l1^2-(foot_q1_dist/2)^2));      % distance from foot_q1_dist  to knee  
        slope=(0-q1_Position(2))/(x(3)+q1_Position(1));
        deg=atand(slope);

        if (x(3)>q1_Position(1))
            knee=[((mid_Hip_x+x(3))+(-c*cosd(90+deg))) ((mid_Hip_y)+(-c*sind(90+deg)))]; %Knee location
        else
            knee=[(mid_Hip_x+x(3))+(c*cosd(90+deg)) (mid_Hip_y)+(c*sind(90+deg))];
        end

        x1=[q1_Position(1) knee(1)]; %Q1 to knee
        y1=[q1_Position(2) knee(2)];
        x2=[knee(1) x(3)];          %knee to floor
        y2=[knee(2) 0];
        
        %Funtion to determine leg angles:
        [deg1(angleCount),deg2(angleCount)]=MeasureAngle (x1, y1, x2, y2);
        ground_Touch(touch_number)=x(3);
        touch_number=touch_number+1;
        %Ploting leg:
        plot  ([knee(1) q1_Position(1)], [knee(2) q1_Position(2)],'color',[.3 .3 .3 .9],'linewidth',3,'LineStyle','-.');
        hold on
        plot ([knee(1) x(3)], [knee(2) 0],'color',[.3 .3 .3 .9],'linewidth',3,'LineStyle','-.');
        hold on
    end
    angleCount=angleCount+1;
    
end

xlim(xlimit1);
ylim(ylimit1);
pbaspect([1 (ylimit1(1)-ylimit1(2))/(xlimit1(1)-xlimit1(2)) 1])
% title('Leg position')
xlabel('cm')
ylabel('cm') 
hold on

degs=[deg1;deg2]'
% figure
% ground_Touch=flip(ground_Touch);

%Funtion for fesible force sets
Tendon_Limb_Design(degs,Motor_Force,ground_Touch,force_figure_scale,0)
