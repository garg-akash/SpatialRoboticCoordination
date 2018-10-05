%To generate fig: min_dist_to_obst&goal.fig
function[vx_out, vy_out, x_out, y_out] = call_mpc(n,delt,robo_start,robo_px,robo_py,obs_py,critical_x,robo_v_start,id_1,id_2)
% n is mpc horizon
% delt is the time duration of 1 timestep, time for 1 planning horizon
% n*delt
%robo_start is the initial start location for robot
%robo_dest is the location of robot from current to n steps ahead
%robo_px,py are the inital x y coord of robots from current to n steps ahead
%critical_x contain x coordinates of points where robot and onstacle meet
%robo_v_start is the velocity of robot at n=1 (start of mpc horizon)

robo_dest_x = robo_px(n);
robo_dest_y = robo_py(n);

%% Getting current velocity of the robot
V0x = robo_v_start(1);
V0y = robo_v_start(2);

%% Amount of change in velocity allowed
del_Vx= 1*delt;
del_Vy= 1*delt;

%% Saving cvx_optval
optval_mat = 0;
q = 2;
optval_mat(1) = 0;
%% Let's start MPC
l = 0;
no_of_iter = 2;

while l < no_of_iter
    tic;
    cvx_begin quiet
    variables Vx(n) Vy(n) %Velocities
    %variables Px(n) Py(n) %locations at each timestep
    
    for i = 1:n
        Px(i)  =  sum(Vx(1:i))*delt + robo_start(1);
        Py(i)  =  sum(Vy(1:i))*delt + robo_start(2);
    end

    %%Cost function
    cost = 0;
    for i=1:n
        if(robo_px(i)>critical_x(1) && robo_px(i)<critical_x(2)) %This will not work for generic cases
            cost = cost + (Px(i) - robo_px(i))^2 + (Py(i) - obs_py(i))^2; %obs_px=robo_px
        end
        if(robo_px(i)>critical_x(2))
            cost = cost + (Px(i) - robo_px(i))^2 + (Py(i) - robo_py(i))^2;
        end
    end 
%     cost = (Px(n) - robo_dest_x)^2 + (Py(n) - robo_dest_y)^2;
%     cost = cost + (Px(3*n/4) - robo_px(3*n/4))^2 + (Py(3*n/4) - robo_py(3*n/4))^2;
%     cost = cost + (Px(n/2) - robo_px(2*n/4))^2 + (Py(n/2) - robo_py(2*n/4))^2;    
%     cost = cost + (Px(n/4) - robo_px(n/4))^2 + (Py(n/4) - robo_py(n/4))^2 ;
    minimise(cost);
    
    %%Constraints
%     for i=1:n
%         if(robo_px(i)>critical_x(1) && robo_px(i)<critical_x(2)) %This will not work for generic cases
%             robo_py(i) <= Py(i)
% %             Py(i) <= obs_py(i); 
%         end
%     end
    
    %% The velocity during initialization shouldn't be much higher
    V0x-del_Vx <= Vx(1) <= V0x + del_Vx;
    V0y-del_Vy <= Vy(1) <= V0y + del_Vy;
    
    %% Ensuring that subsequent X vel commands do not have difference more than del_Vx
    -del_Vx <= Vx(2:n) - Vx(1:n-1) <= del_Vx;
    
    %% Ensuring that subsequent Y vel commands do not have difference more than del_Vy
    -del_Vy <= Vy(2:n) - Vy(1:n-1) <= del_Vy;
    cvx_end
    
    x_out = Px;
    y_out = Py;
    vx_out = Vx;
    vy_out = Vy;
    
    clear Vx Vy Px Py
    l = l + 1;
    if l ~= 0
        q;
        optval_mat(q) = cvx_optval;
        q = q+1;
    end
    optval_mat(q-1) - optval_mat(q-2);
    if (optval_mat(q-1) - optval_mat(q-2) > 0.1) || (optval_mat(q-1) - optval_mat(q-2) < -0.1)
        l = l - 1;
    end
    toc;
end
