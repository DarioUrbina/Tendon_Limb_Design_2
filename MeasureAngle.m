%% MeasureAngle
% for getting joint angles
% q1 0 deg when it is pointing down. Clock wise movement: negative
% knee 0 deg when aligned with upper limb segment. Clock wise movement:
% negative

%[q1, knee]=MeasureAngle (x component of q1 and knee, 
%                           y component of q1 and knee,
%                           x component of knee and floor,
%                           y component of knee and floor)
%%
function [a,b_ang]=MeasureAngle (x1, y1, x2, y2)


    %%%%%%% Plots %%%%%%%%%%Uncomment this section to test with
    %%%%%%% TestingMeasuringAngle.m
    %     figure
    %     plot(x1,y1);                                  %Ploting a desplaced a triangle
    %     hold on
    % %     plot(x2+x1(3),y2-y2(2)); %Perhaps get rid of this would workfor the complete project                     %Ploting a desplaced a triangle 
    %     plot(x2,y2); 
    %     xlim([-10 10]);
    %     ylim([-1 16]);
    %     grid on
    %     slope_b1_c1=0;
    %     slope_b2_c2=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (x1(1)==x1(2))
        x1(2)=x1(2)+.01;
    end
    if (x2(1)==x2(2))
        x2(2)=x2(2)+.01;
    end

    slope_a=(atand((y1(2)-y1(1))/(x1(2)-x1(1))));
    slope_b=(atand((y2(2)-y2(1))/(x2(2)-x2(1))));

    %Angles are calculated depending on possible leg positions:
    if slope_b>0
        b=-(90-(slope_b));
    else
        b=(90+(slope_b));
    end


    if slope_a>0
        a=-(90-(slope_a));
        if(((y1(1))-y1(2))>(y2(1)-y2(2)))
             b_ang=b-a;
             flag="1";
        else
            if slope_b>0
            b_ang=180-(-b+90+(90-abs(a)));
            flag="2";
            else
            b_ang=180-(-b+90+(90-abs(a)));
            flag="3";
            end
        end

    else
        a=(90+(slope_a));
        if(((y1(1))-y1(2))>(y2(1)-y2(2)))
            b_ang=b-a;
            flag="4";
        else
            if slope_b>0
            b_ang=-(180-(90-abs(a)+90+b))
            flag="5";
            else
            b_ang=-(180-(90-abs(a)+90+b));
            flag="6";
            end
        end    
    end
    
end
