function[vx_out, vy_out, x_out, y_out] = call_mpc(n,delt,robo_start,robo_px,robo_py,obs_py,critical_x,robo_v_start,id_1,id_2,id_count,w1,w2)
%To generate fig: min_dist_to_obst&goal.fig

% n is mpc horizon
% delt is the time duration of 1 timestep, time for 1 planning horizon
% n*delt
%robo_start is the initial start location for robot
%robo_dest is the location of robot from current to n steps ahead
%robo_px,py are the inital x y coord of robots from current to n steps ahead
%critical_x contain x coordinates of points where robot and onstacle meet
%robo_v_start is the velocity of robot at n=1 (start of mpc horizon)

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
    cost_2 = 0; 
    flag = zeros(1,n);
    %id_count prevents setting flag to 1 in interval of extended path
    for i=1:n
        if(robo_px(i)>critical_x(1) && robo_px(i)<critical_x(2) && id_count<id_2) %This will not work for generic cases
            flag(i) = 1;
        end
    end 
    %I need to consider area from start_id to end_id for the present horizon 

    cost = (Px(n) - robo_px(n))^2 + (Py(n) - robo_py(n))^2;
    cost = cost + (Px(3*n/4) - robo_px(3*n/4))^2 + (Py(3*n/4) - robo_py(3*n/4))^2;
    cost = cost + (Px(n/2) - robo_px(2*n/4))^2 + (Py(n/2) - robo_py(2*n/4))^2;    
    cost = cost + (Px(n/4) - robo_px(n/4))^2 + (Py(n/4) - robo_py(n/4))^2 ;
%     minimise(cost);

    start_id = min(find(flag));
    end_id = max(find(flag));
    disp(flag)
    disp(start_id)
    disp(end_id)
    disp(id_count)
%     area_obs = trapz(robo_px(start_id:end_id),obs_py(start_id:end_id));
%     area_init = trapz(robo_px(start_id:end_id),robo_py(start_id:end_id));
%     disp(area_init)
%     area_opt = trapz(robo_px(start_id:end_id),Py(start_id:end_id));
%     disp(area_opt)
%     -area_init <= -area_opt;
%     area_opt <= area_init;

%     sum(Py(start_id:end_id)) <= sum(robo_py(start_id:end_id));
    cost_2 = sum(Py(start_id:end_id)) - sum(robo_py(start_id:end_id));
    
    cost_total = w1*cost + w2*cost_2;
    minimise(cost_total)
        
    %%Constraints
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
