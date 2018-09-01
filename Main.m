
% clear all;close all;clc
%% Paremeters
l1=8;                  %Determines leg size, in this case we asume that upper and lower limb segments are the same size
q1_Position=[0 15];    %Modify these parameter dependig on the leg size
xlimit1=[-8 8];        %Figure 1 Limits
ylimit1=[-5 17];       %Figure 2 Limits
xlimit2=[-15 100];     %Modify these parameter dependig on the feasible forse size
ylimit2=[-30 40];      
firstTouchPoint=20;    %Modify this parameter dependig on the leg size
                        %Include forces and Moment Arm Matrix here
Motor_Force=49*.80;
force_figure_scale=.15;
rr=1;
%%

angleCount=0;

figure

%%
for i=1:3.6:45
    
    %     i
        x(1) = q1_Position(1);
        y(1) = q1_Position(2);
        x(2) = q1_Position(1);
        y(2) = 0;
        x(3) = firstTouchPoint-i; 
        y(3) = 0;
        x(4) = q1_Position(1); 
        y(4) = q1_Position(2);
    %    
        mid_Hip_x=(q1_Position(1)-x(3))/2;
        mid_Hip_y=(q1_Position(2)-y(3))/2;

        if (x(3)<0)
        foot_q1_dist=sqrt(((x(1)+x(3))^2)+(y(1)-y(2))^2);
        else
        foot_q1_dist=sqrt(((x(1)-x(3))^2)+(y(1)-y(2))^2);   
        end
        max_dist=l1*2;
        c=sqrt((l1^2-(foot_q1_dist/2)^2));

        slope=(y(3)-q1_Position(2))/(x(3)+q1_Position(1));
        deg=atand(slope);

        knee_push=[(mid_Hip_x+x(3))+(c*cosd(90+deg)) (mid_Hip_y+y(3))+(c*sind(90+deg))];
        knee_pull=[((mid_Hip_x+x(3))+(-c*cosd(90+deg))) ((mid_Hip_y+y(3))+(-c*sind(90+deg)))];

    %     slope_l1=atand((knee_push(2)-y(1))/(knee_push(1)-x(1));

    if(foot_q1_dist<=(l1*2))
        angleCount=angleCount+1;
        if (x(3)>q1_Position(1))

%             subplot (4,1,1) 
%             subplot (2,1,1)
            subplot (1,1,1) 
             plot ([knee_pull(1) x(3)], [knee_pull(2) y(3)],'color',[.3 .3 .3 .9],'linewidth',3,'LineStyle','-.');

            hold on
%             subplot (4,1,1)
%             subplot (2,1,1)
            subplot (1,1,1) 
             plot  ([knee_pull(1) x(1)], [knee_pull(2) y(1)],'color',[.3 .3 .3 .9],'linewidth',3,'LineStyle','-.');

            x1=[q1_Position(1) real(knee_pull(1))];
            y1=[q1_Position(2) real(knee_pull(2))];

            x2=[real(knee_pull(1)) real(knee_pull(1))];
            y2=[x(3) y(3)];
%              x2=[0 0              x(3)-knee_pull(1)     0];
%              y2=[0 knee_pull(2)   0                     0];


            [deg1(angleCount),deg2(angleCount)]=MeasureAngle (x1, y1, x2, y2);
%              if (rr==2)
%             deg2(angleCount)=180-deg2(angleCount)
%              end
%             rr=2;

        else

%             subplot (4,1,1);
%             subplot (2,1,1) 
            subplot (1,1,1) 
             plot([knee_push(1) x(3)], [knee_push(2) y(3)],'color',[.3 .3 .3 .9],'linewidth',3,'LineStyle','-.');
            hold on
%             subplot (4,1,1);
%             subplot (2,1,1) 
            subplot (1,1,1) 
             plot ([knee_push(1) x(1)], [knee_push(2) y(1)],'color',[.3 .3 .3 .9],'linewidth',3,'LineStyle','-.');

            x1=[q1_Position(1) real(knee_push(1))];
            y1=[q1_Position(2) real(knee_push(2))];

            x2=[real(knee_push(1)) real(knee_push(1))];
            y2=[x(3) y(3)];

%              x2=[0 0              x(3)-knee_push(1)     0];
%              y2=[0 knee_push(2)   0                     0];
             
            [deg1(angleCount),deg2(angleCount)]=MeasureAngle (x1, y1, x2, y2);
%              deg2(angleCount)=180-deg2(angleCount)

            if (x(3)<knee_push(1))
                deg2(angleCount)=-deg2(angleCount)
            end
        end
        ground_Touch(angleCount)=x(3);

    end
end

xlim(xlimit1);
ylim(ylimit1);
pbaspect([1 (ylimit1(1)-ylimit1(2))/(xlimit1(1)-xlimit1(2)) 1])
% title('Leg position')
xlabel('cm')
ylabel('cm') 
hold on

% subplot (4,1,2)
% plot (deg1)
% ylim([-100 100]);
% title('deg1')
% xlabel('Leg position')
% ylabel('deg') 
% 
% 
% subplot (4,1,3)
% plot (deg2)
% ylim([-100 100]);
% title('deg2')
% xlabel('Leg position')
% ylabel('deg')
     
degs=[deg1;deg2]'
% subplot (4,1,4)

hold on
subplot (1,1,1) 
ground_Touch=flip(ground_Touch);
Tendon_Limb_Design(degs,Motor_Force,ground_Touch,force_figure_scale)
xlim(xlimit1);
ylim(ylimit1);
pbaspect([1 (ylimit1(1)-ylimit1(2))/(xlimit1(1)-xlimit1(2)) 1])

% hold on
figure
% subplot (2,1,2) 

Tendon_Limb_Design(degs,Motor_Force,0,1)
xlim(xlimit2);
ylim(ylimit2);
pbaspect([1 (ylimit2(1)-ylimit2(2))/(xlimit2(1)-xlimit2(2)) 1])






    