function[vx_out, vy_out, x_out, y_out, t_out] = footprint_call_mpc(n,delt,robo_start,robo_px,robo_py ...
                                                ,obs_py,robo_v_start,robo_t_start,l_1,b_1,obs_t,l_2,b_2,hinit_2)
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

%% Getting current velocity & theta of the robot
V0x = robo_v_start(1);
V0y = robo_v_start(2);
t0 = robo_t_start;

%% Amount of change in velocity & theta allowed
%delt for now is 0.5
del_Vx = 1*delt;
del_Vy = 1*delt;
del_th = (1/5)*delt;

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
    variables Vx(n) Vy(n) t(n) %Velocities & theta
    %variables Px(n) Py(n) %locations at each timestep
    
    for i = 1:n
        Px(i)  =  sum(Vx(1:i))*delt + robo_start(1);
        Py(i)  =  sum(Vy(1:i))*delt + robo_start(2);
%         tfinal(i) = t(i) + hinit_2;
    end

    %%Cost function
    cost = 0;
    cost = (Px(n) - robo_dest_x)^2 + (Py(n) - robo_dest_y)^2;
    cost = cost + (Px(3*n/4) - robo_px(3*n/4))^2 + (Py(3*n/4) - robo_py(3*n/4))^2;
    cost = cost + (Px(n/2) - robo_px(2*n/4))^2 + (Py(n/2) - robo_py(2*n/4))^2;    
    cost = cost + (Px(n/4) - robo_px(n/4))^2 + (Py(n/4) - robo_py(n/4))^2 ;
    minimise(cost);
    
    %%Constraints
    for i=1:n
        [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,obs_t(i),robo_px(i),obs_py(i));
%         [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,-tfinal(i),Px(i),Py(i));
        [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,obs_t(i)-2*pi,Px(i),Py(i));
%         Poly_1 = [AG_1,BG_1,CG_1,DG_1];
%         Poly_2 = [AG_2;BG_2;CG_2;DG_2];
%         [F] = call_minkowski(Poly_1,Poly_2);
%         [F_trans] = call_trans(Poly_1,Poly_2,F);
%         mx_2 = mean(Poly_2(:,1))
%         my_2 = mean(Poly_2(:,2))
%         [in,on] = inpolygon(mx_2,my_2,F_trans(1,1:end),F_trans(2,1:end));
%         in == 0;
    end
    
    %% The velocity & theta during initialization shouldn't be much higher
    V0x-del_Vx <= Vx(1) <= V0x + del_Vx;
    V0y-del_Vy <= Vy(1) <= V0y + del_Vy;
    
    t0 - del_th <= t(1) <= t0 + del_th;
    
    %% Ensuring that subsequent X vel commands do not have difference more than del_Vx
    -del_Vx <= Vx(2:n) - Vx(1:n-1) <= del_Vx;
    
    %% Ensuring that subsequent Y vel commands do not have difference more than del_Vy
    -del_Vy <= Vy(2:n) - Vy(1:n-1) <= del_Vy;
    
    %% Ensuring that subsequent theta commands do not have difference more than del_th
    -del_th <= t(2:n) - t(1:n-1) <= del_th;
    cvx_end
    
    x_out = Px;
    y_out = Py;
    vx_out = Vx;
    vy_out = Vy;
    t_out = t;
    
    clear Vx Vy Px Py t
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