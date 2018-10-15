clc;
% clear;
close all;
warning off;
c_1 = [0,3.5];
c_2 = [0,-3.5];
% c_1 = [-3,4];
% c_2 = [4,1];
r = 4;
total_t = 40;
avg_vel = 2*pi*r/total_t;

th = pi:pi/39:2*pi;
del_t = total_t/(2*length(th));
steps = total_t/del_t;

%%footprint of obstacle/unit 1
hinit_1 = 3*pi/2; % initial heading for unit 1
b_1 = 0.75;
l_1 = 1.5;

%%footprint of robot/unit 2
hinit_2 = pi/2; % initial heading for unit 2
b_2 = 0.75;
l_2 = 1.5; 

Xunit_1=[];
Yunit_1=[];
Hfinal_1=[];
Xunit_2=[];
Yunit_2=[];
Hfinal_2=[];
Vxunit_2=[];
Vyunit_2=[];
xnow_1 = c_1(1)-r;
ynow_1 = c_1(2);
xnow_2 = c_2(1)-r;
ynow_2 = c_2(2);

for i=1:length(th)
    xunit_1 = xnow_1;
    yunit_1 = ynow_1;
    Xunit_1 = [Xunit_1,xunit_1];
    Yunit_1 = [Yunit_1,yunit_1];
    hfinal_1 = th(i) + hinit_1;
    Hfinal_1 = [Hfinal_1,hfinal_1];
    
    xunit_2 = xnow_2;
    yunit_2 = ynow_2;
    Xunit_2 = [Xunit_2,xunit_2];
    Yunit_2 = [Yunit_2,yunit_2];
    hfinal_2 = th(i) + hinit_2;
    Hfinal_2 = [Hfinal_2,hfinal_2];
    
    xdel_1 = avg_vel*(-sin(th(i)))*del_t;
    ydel_1 = avg_vel*(cos(th(i)))*del_t;
    xnow_1 = xdel_1 + xunit_1;
    ynow_1 = ydel_1 + yunit_1;
    
    xdel_2 = -avg_vel*(-sin(2*pi-th(i)))*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2 = -avg_vel*(cos(2*pi-th(i)))*del_t;
    xnow_2 = xdel_2 + xunit_2;
    ynow_2 = ydel_2 + yunit_2;
    Vxunit_2 = [Vxunit_2;-avg_vel*(-sin(2*pi-th(i)))];
    Vyunit_2 = [Vyunit_2;-avg_vel*(cos(2*pi-th(i)))];
end 

%% plot the initial trjectories 
% cal_initTraj_plot(Xunit_1,Xunit_2,Yunit_1,Yunit_2,th,hinit_1,hinit_2,l_1,b_1,l_2,b_2)
% call_sim(Xunit_1,Xunit_2,Yunit_1,Yunit_2,th,hinit_1,hinit_2,l_1,b_1,l_2,b_2);

% figure
% poly1 = plot(Xunit_1,Yunit_1);
% hold on
% poly2 = plot(Xunit_2,Yunit_2);
% % y_d = [Yunit_2(Yunit_2<Yunit_1) Yunit_1(Yunit_1<Yunit_2)]; 
% % plot(Xunit_1,y_d,'k-o')
% x_inter = [Xunit_1(Yunit_2>Yunit_1), fliplr(Xunit_1(Yunit_1<Yunit_2))];
% y_inter1 = [Yunit_2(Yunit_2>Yunit_1)]; 
% y_inter2 = [Yunit_1(Yunit_1<Yunit_2)];
% inBetween = [y_inter1, fliplr(y_inter2)];
% fill(x_inter,inBetween, 'g');
% hold off
% % area_int  = trapz(inBetween)
% 
% area_upper = trapz(Xunit_1(Yunit_2>Yunit_1),y_inter1);
% area_below = trapz(Xunit_1(Yunit_2>Yunit_1),y_inter2);
% area_total = area_upper - area_below;
% % polyout = intersect(poly1,poly2)
% 
% %%Optimize the overlap area
% %Robo is nothing but unit_2
% critical_x = [x_inter(1),x_inter(end/2)];
%extended points are for mpc loop as the robo_dest goes beyond 1xlength(th)

Xunit_2 = robo_traj(:,1)';
Yunit_2 = robo_traj(:,2)';
Hfinal_2 = tidk';
Xunit_1_ex = Xunit_1;
Yunit_1_ex = Yunit_1;
Hfinal_1_ex = Hfinal_1;

Xunit_2_ex = Xunit_2;
Yunit_2_ex = Yunit_2;
Hfinal_2_ex = Hfinal_2;
th_ex = 0:pi/39:pi;

xunit_1_ex = xunit_1;
yunit_1_ex = yunit_1;
xunit_2_ex = xunit_2;
yunit_2_ex = yunit_2;
for i=2:length(th_ex) %i=1 corresponds to th_ex=0 which is same as th=2*pi
    xdel_1_ex = avg_vel*(-sin(th_ex(i)))*del_t;
    ydel_1_ex = avg_vel*(cos(th_ex(i)))*del_t;
    
    xdel_2_ex = -avg_vel*(-sin(2*pi-th_ex(i)))*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2_ex = -avg_vel*(cos(2*pi-th_ex(i)))*del_t;
    
    xnow_1_ex = xdel_1_ex + xunit_1_ex;
    ynow_1_ex = ydel_1_ex + yunit_1_ex;
    xunit_1_ex = xnow_1_ex;
    yunit_1_ex = ynow_1_ex;
    Xunit_1_ex = [Xunit_1_ex,xunit_1_ex];
    Yunit_1_ex = [Yunit_1_ex,yunit_1_ex];
    Hfinal_1_ex = [Hfinal_1_ex,(th_ex(i) + hinit_1)];
    xnow_2_ex = xdel_2_ex + xunit_2_ex;
    ynow_2_ex = ydel_2_ex + yunit_2_ex;
    xunit_2_ex = xnow_2_ex;
    yunit_2_ex = ynow_2_ex;
    Xunit_2_ex = [Xunit_2_ex,xunit_2_ex];
    Yunit_2_ex = [Yunit_2_ex,yunit_2_ex];
    Hfinal_2_ex = [Hfinal_2_ex,(th_ex(i) + hinit_2)];
end

%del_t = 1.0; % Time duration between two timesteps delt
n = 12; % n has to be divisible by 4
robo_start  = [Xunit_2(1),Yunit_2(1)];
robo_end  = [Xunit_2(end),Yunit_2(end)];
robo_v_start = [0,0]; % start velocity of the robo
robo_t_start = th(1)+hinit_2;
robo_traj = [];
robo_vel = [];
robo_t = [];

for i = 1:length(th)
    [Vx Vy Px Py tfinal] = footprint_call_mpc(n,del_t,robo_start,Xunit_2_ex(i:n+i-1),Yunit_2_ex(i:n+i-1),Yunit_1_ex(i:n+i-1) ...
                   ,robo_v_start,robo_t_start,l_1,b_1,Hfinal_1_ex(i:n+i-1),l_2,b_2,hinit_2,Hfinal_2_ex(i:n+i-1));
    robo_start =  [Px(1) Py(1)];
    robo_v_start = [Vx(1) Vy(1)];
    robo_t_start = tfinal(1);
    robo_traj = [robo_traj;robo_start]; %stores the optimized trajectory of the robot 
    robo_vel = [robo_vel;robo_v_start]; %stores the optimized velocity of the robot
    robo_t = [robo_t;robo_t_start];
end
tidk = atan2(robo_vel(:,2),robo_vel(:,1));
%Heading of robot is tidk but we send -tidk bcoz in rectangle_plot we send
%-ve. So -(-) becomes +ve
cal_finalTraj_plot(Xunit_1,Yunit_1,Yunit_2,robo_traj',th,hinit_1,-tidk,l_1,b_1,l_2,b_2)
% cal_finalTraj_plot(Xunit_1,Yunit_1,Yunit_2,robo_traj',th,hinit_1,robo_t,l_1,b_1,l_2,b_2)
