clc;
clear;
close all;
c_1 = [0,3];
c_2 = [0,-3];
% c_1 = [-3,4];
% c_2 = [4,1];
r = 4;
total_t = 40;
avg_vel = 2*pi*r/total_t;

th = pi:pi/39:2*pi;
del_t = total_t/(2*length(th));
steps = total_t/del_t;

Xunit_1=[];
Yunit_1=[];
Xunit_2=[];
Yunit_2=[];
Vxunit_2=[];
Vyunit_2=[];
xnow_1 = c_1(1)-r;
ynow_1 = c_1(2);
xnow_2 = c_2(1)-r;
ynow_2 = c_2(2);

axis([-5  5  -5  5])
for i=1:length(th)
    xunit_1 = xnow_1;
    yunit_1 = ynow_1;
    Xunit_1 = [Xunit_1,xunit_1];
    Yunit_1 = [Yunit_1,yunit_1];
    
    xunit_2 = xnow_2;
    yunit_2 = ynow_2;
    Xunit_2 = [Xunit_2,xunit_2];
    Yunit_2 = [Yunit_2,yunit_2];
    
    hold on
    h_1 = plot(xunit_1, yunit_1,'bp');
    h_2 = plot(xunit_2, yunit_2,'rp');
    pause(0.5);
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
figure
poly1 = plot(Xunit_1,Yunit_1);
hold on
poly2 = plot(Xunit_2,Yunit_2);
% y_d = [Yunit_2(Yunit_2<Yunit_1) Yunit_1(Yunit_1<Yunit_2)]; 
% plot(Xunit_1,y_d,'k-o')
x_inter = [Xunit_1(Yunit_2>Yunit_1), fliplr(Xunit_1(Yunit_1<Yunit_2))];
y_inter1 = [Yunit_2(Yunit_2>Yunit_1)]; 
y_inter2 = [Yunit_1(Yunit_1<Yunit_2)];
inBetween = [y_inter1, fliplr(y_inter2)];
fill(x_inter,inBetween, 'g');
hold off
% area_int  = trapz(inBetween)

area_upper = trapz(y_inter1);
area_below = trapz(abs(y_inter2));
area_total = area_upper + area_below;
% polyout = intersect(poly1,poly2)

%%Optimize the overlap area
%Robo is nothing but unit_2
robo_start  = [Xunit_2(1),Yunit_2(1)];
robo_end  = [Xunit_2(end),Yunit_2(end)];
critical_x = [x_inter(1),x_inter(end/2)];
%extended points are for mpc loop as the robo_dest goes beyond 1xlength(th)
Xunit_1_ex = Xunit_1;
Yunit_1_ex = Yunit_1;

Xunit_2_ex = Xunit_2;
Yunit_2_ex = Yunit_2;
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
    xnow_2_ex = xdel_2_ex + xunit_2_ex;
    ynow_2_ex = ydel_2_ex + yunit_2_ex;
    xunit_2_ex = xnow_2_ex;
    yunit_2_ex = ynow_2_ex;
    Xunit_2_ex = [Xunit_2_ex,xunit_2_ex];
    Yunit_2_ex = [Yunit_2_ex,yunit_2_ex];
end

%del_t = 1.0; % Time duration between two timesteps delt
n = 12; % n has to be divisible by 4
robo_v_start = [0,0]; % start velocity of the robo

robo_traj = [];
robo_vel = [];

for i = 1:length(th)
%     [Vx Vy Vz Px Py Pz] = compute_trajectory(robo_start,robo_v_start, [Ox1(i),Oy1(i),Oz1(i)], vel_obs1, [Ox2(i),Oy2(i),Oz2(i)], vel_obs2, [Ox3(i),Oy3(i),Oz3(i)], vel_obs3, dest_location, n, del_t, sense_R, avg_vel);
    
    [Vx Vy Px Py] = call_mpc(n,del_t,robo_start,Xunit_2_ex(i:n+i-1),Yunit_2_ex(i:n+i-1),Yunit_1_ex(i:n+i-1),critical_x,robo_v_start);
    robo_start =  [Px(1) Py(1)];

    robo_v_start = [Vx(1) Vy(1)];
    robo_traj = [robo_traj;robo_start]; %stores the optimized trajectory of the robot 
    robo_vel = [robo_vel;robo_v_start]; %stores the optimized velocity of the robot
end

call_plot(robo_traj,Xunit_1,Yunit_1,Xunit_2,Yunit_2,robo_vel,Vxunit_2,Vyunit_2);
