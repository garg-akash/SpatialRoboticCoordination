function [F_trans_fig, poly_mean_fig] = call_collision(Poly_2,F_trans)

mx_2 = mean(Poly_2(:,1))
my_2 = mean(Poly_2(:,2))

figure(3)
hold on
axis([-8 8 -8 8]);
F_trans_fig = plot(F_trans(1,1:end),F_trans(2,1:end),'g*-');
hold on
[in,on] = inpolygon(mx_2,my_2,F_trans(1,1:end),F_trans(2,1:end));
if (in || on)
    poly_mean_fig = plot(mx_2,my_2,'kp');
else
    poly_mean_fig = plot(mx_2,my_2,'rp');
end