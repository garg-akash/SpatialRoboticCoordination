function[] = cal_initTraj_plot(Xunit_1,Xunit_2,Yunit_1,Yunit_2,th,hinit_1,hinit_2,l_1,b_1,l_2,b_2)

for i=1:length(th)   
    figure(1);
    hold on
    set(gca,'XLim', [floor(min([Xunit_1, Xunit_2]))-5, floor(max([Xunit_1,Xunit_2]))+6] ...
        ,'YLim', [floor(min([Yunit_1, Yunit_2]))-4, floor(max([Yunit_1, Yunit_2]))+4]);
    hold on

    p11 = plot(Xunit_1(i), Yunit_1(i),'bp');
    p21 = plot(Xunit_2(i), Yunit_2(i),'rp');
    
    hfinal_1 = th(i) + hinit_1;
    hfinal_2 = th(i) + hinit_2;
    
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,hfinal_1,Xunit_1(i),Yunit_1(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,-hfinal_2,Xunit_2(i),Yunit_2(i));
    
    %% plot footprint for obstacle
    p12 = line([AG_1(1);BG_1(1)],[AG_1(2);BG_1(2)]);
    hold on;
    p13 = line([BG_1(1);CG_1(1)],[BG_1(2);CG_1(2)]);
    hold on;
    p14 = line([CG_1(1);DG_1(1)],[CG_1(2);DG_1(2)]);
    hold on;
    p15 = line([DG_1(1);AG_1(1)],[DG_1(2);AG_1(2)]);
    hold on;
    mid1_1=(AG_1+BG_1)/2;
    mid2_1=(CG_1+DG_1)/2;
    p16 = line([mid1_1(1);mid2_1(1)],[mid1_1(2);mid2_1(2)],'Color','red','LineStyle','--');
    
    %% plot footprint for robot
    p22 = line([AG_2(1);BG_2(1)],[AG_2(2);BG_2(2)]);
    hold on;
    p23 = line([BG_2(1);CG_2(1)],[BG_2(2);CG_2(2)]);
    hold on;
    p24 = line([CG_2(1);DG_2(1)],[CG_2(2);DG_2(2)]);
    hold on;
    p25 = line([DG_2(1);AG_2(1)],[DG_2(2);AG_2(2)]);
    hold on;
    mid1_2=(AG_2+BG_2)/2;
    mid2_2=(CG_2+DG_2)/2;
    p26 = line([mid1_2(1);mid2_2(1)],[mid1_2(2);mid2_2(2)],'Color','red','LineStyle','--');
    
    %Poly_1 is for the obstacle/unit_1, Poly_2 is for the robot/unit_2  
    Poly_1 = [AG_1;BG_1;CG_1;DG_1];
    Poly_2 = [AG_2;BG_2;CG_2;DG_2];
%     plot(Poly_2(:,1),Poly_2(:,2));
    [F] = call_minkowski(Poly_1,Poly_2);
    %minkow = plot(F(:,1),F(:,2),'b');
    
    [F_trans] = call_trans(Poly_1,Poly_2,F);
    figure(2)
    hold on
    set(gca,'XLim', [floor(min([Xunit_1, Xunit_2]))-5, floor(max([Xunit_1,Xunit_2]))+6] ...
        ,'YLim', [floor(min([Yunit_1, Yunit_2]))-4, floor(max([Yunit_1, Yunit_2]))+4]);
    hold on
    minkow_init = plot(F(:,1),F(:,2),'b*-');
    hold on
    minkow_trans = plot(F_trans(1,1:end),F_trans(2,1:end),'g*-');
    
 
    [F_trans_fig, poly_mean_fig] = call_collision(Poly_2,F_trans);
    
    pause(0.50);
    if (i ~= length(th))
        delete(p12);delete(p13);delete(p14);delete(p15);delete(p16);
        delete(p22);delete(p23);delete(p24);delete(p25);delete(p26);
        %delete(minkow);
        delete(minkow_init);delete(minkow_trans);
        delete(F_trans_fig);delete(poly_mean_fig);
    end
end