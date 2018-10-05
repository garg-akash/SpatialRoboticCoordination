function[] = call_plot(robo_traj,Xunit_1,Yunit_1,Xunit_2,Yunit_2,robo_vel,Vxunit_2,Vyunit_2,w1,w2)

figure
subplot(2,2,1)
plot(robo_traj(:,1),robo_traj(:,2));
hold on
plot(Xunit_2,Yunit_2);
plot(Xunit_1,Yunit_1);
title('Trjaectory Plot')
legend(sprintf('W1 = %f & W2 = %f', w1, w2),'Initial Trajectroty','Obstacle Trajectroty')
hold off

subplot(2,2,2)
plot(Vxunit_2);
hold on
plot(robo_vel(:,1));
title('Velocity_X Profile Plot')
legend('Initial velocity','Optimized velocity')
hold off

subplot(2,2,3)
plot(Vyunit_2);
hold on
plot(robo_vel(:,2));
title('Velocity_Y Profile Plot')
legend('Initial velocity','Optimized velocity')
hold off