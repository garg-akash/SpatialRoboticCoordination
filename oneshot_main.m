%This version deals with one shot planning ie, consider full time as mpc horizon 
%It finds trajectory for variable waypoints 
%Here I am finding intersection points of the trajectories 
%and taking care of discretization error 
clc;
clear;
close all;
c_1 = [0;3];
c_2 = [0;-3];
r = 4;
% c_1 = [-3;4];
% c_2 = [4;1];
r1 = r;
r2 = r;
d2 = sum((c_2-c_1).^2);
P0 = (c_1+c_2)/2+(r1^2-r2^2)/d2/2*(c_2-c_1);
t = ((r1+r2)^2-d2)*(d2-(r2-r1)^2);
if t <= 0
    fprintf('The circles don''t intersect.\n')
else
    T = sqrt(t)/d2/2*[0 -1;1 0]*(c_2-c_1);
    Pa = P0 - T; % Pa and Pb are circles' intersection points
    Pb = P0 + T;
end
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
    pause(0.3);
    xdel_1 = avg_vel*(-sin(th(i)))*del_t;
    ydel_1 = avg_vel*(cos(th(i)))*del_t;
    xnow_1 = xdel_1 + xunit_1;
    ynow_1 = ydel_1 + yunit_1;
    
    xdel_2 = -avg_vel*(-sin(2*pi-th(i)))*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2 = -avg_vel*(cos(2*pi-th(i)))*del_t;
    xnow_2 = xdel_2 + xunit_2;
    ynow_2 = ydel_2 + yunit_2;
    Vxunit_2 = [Vxunit_2;-avg_vel*(-sin(2*pi-th(i)))]; %Vxunit_2 is used to plot the velocity profile
    Vyunit_2 = [Vyunit_2;-avg_vel*(cos(2*pi-th(i)))];
end
figure
poly1 = plot(Xunit_1,Yunit_1);
hold on
poly2 = plot(Xunit_2,Yunit_2);
% y_d = [Yunit_2(Yunit_2<Yunit_1) Yunit_1(Yunit_1<Yunit_2)]; 
% plot(Xunit_1,y_d,'k-o')

critical_trivial_x = [Pa(1),Pb(1)];
critical_trivial_y = [Pa(2),Pb(2)];
%find index of values closest to critical points/points of inetrsection
[ x_ntreq1, x_id1 ] = min( abs( Xunit_1-critical_trivial_x(1) ) );
[ x_ntreq2, x_id2 ] = min( abs( Xunit_1-critical_trivial_x(2) ) );
%due to discretization error just extend the bounds by 1 on both the sides
if x_id1>1
    x_id1 = x_id1 - 1;
end
if x_id2 < length(Xunit_1)
    x_id2 = x_id2 + 1;
end
x_inter1 = Xunit_1(x_id1:x_id2);
x_inter2 = fliplr(x_inter1);
x_inter = [x_inter1,x_inter2];
% y_inter1 = Yunit_1(Xunit_1>=Pa(1) & Xunit_1<=Pb(1)); 
% y_inter2 = Yunit_2(Xunit_1>=Pa(1) & Xunit_1<=Pb(1));
% inBetween = [(y_inter1),fliplr(y_inter2)];
% fill(x_inter,fliplr(inBetween), 'g');
% hold off
% % area_int  = trapz(inBetween)
% 
% area_upper = trapz(abs(y_inter1));
% area_below = trapz(y_inter2);
% area_total = area_upper + area_below;
% % polyout = intersect(poly1,poly2)

%update the critical points
critical_x = [x_inter1(1),x_inter1(end)];
%%Optimize the overlap area
%Robo is nothing but unit_2
robo_start  = [Xunit_2(1),Yunit_2(1)];
robo_end  = [Xunit_2(end),Yunit_2(end)];

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
n = length(th); % full time horizon  
wp = [1,2,4,5,8,10,20]; %do not count start point in waypoints && wp should divide length(th)
%wp = [1];
robo_v_start = [0,0]; % start velocity of the robo
robo_traj = [];
robo_vel = [];

%for i = 1:length(th)
for i = 1:length(wp)
    [Vx, Vy, Px, Py] = oneshot_call_mpc(wp(i),n,del_t,robo_start,Xunit_2_ex(i:n+i-1),Yunit_2_ex(i:n+i-1),Yunit_1_ex(i:n+i-1),critical_x,robo_v_start);
    figure
    plot(Px,Py);
    hold on
    plot(Xunit_2,Yunit_2);
    plot(Xunit_1,Yunit_1);
    title('Trjaectory Plot')
    legend(sprintf('WP = %f', wp(i)),'Initial Trajectroty','Obstacle Trajectroty')
    fprintf('Called for %d',wp(i))
end

