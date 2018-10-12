function[] = cal_finalTraj_plot(Xunit_1,Yunit_1,Yunit_2,robo_traj,th,hinit_1,robo_t,l_1,b_1,l_2,b_2)
size(robo_traj)

v = VideoWriter('sim.avi', 'Motion JPEG AVI');
v.FrameRate = 4;
open(v);
hfig = figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
pause(2);
hold on
set(gca,'XLim', [floor(min([Xunit_1, robo_traj(1,:)]))-5, floor(max([Xunit_1, robo_traj(1,:)]))+5] ...
    ,'YLim', [floor(min([Yunit_1, robo_traj(2,:)]))-5, floor(max([Yunit_1, robo_traj(2,:)]))+5]);
hold on
for i=1:length(th)   

    p11 = plot(Xunit_1(i), Yunit_1(i),'bp');
    p21 = plot(robo_traj(1,i), robo_traj(2,i),'rp');
    p31 = plot(Xunit_1(i),Yunit_2(i),'kx');
    legend('Obstacle Trajectory','Optimized Robo Traj','Initial Robo Traj');
    
    hfinal_1 = th(i) + hinit_1;
    
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,hfinal_1,Xunit_1(i),Yunit_1(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,-robo_t(i),robo_traj(1,i),robo_traj(2,i));
    
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
    
    M(i) = getframe(hfig);
    writeVideo(v,M(i));
    pause(0.50);
    if (i ~= length(th))
        delete(p12);delete(p13);delete(p14);delete(p15);delete(p16);
        delete(p22);delete(p23);delete(p24);delete(p25);delete(p26);
    end
end
close(v);