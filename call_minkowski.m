function [F] = call_minkowski(Poly_1,Poly_2)
F_old = minkowskiSum(Poly_1,-Poly_2);
x_a = Poly_1(:,1);
y_a = Poly_1(:,2);
x_b = Poly_2(:,1);
y_b = Poly_2(:,2);
Poly_1
Poly_2
F_old
F = F_old;
%To eliminate repeating rows
for i = 1:length(F_old)-1
    if(round(F_old(i,:),2)== round(F_old(i+1,:),2))
        F(i,:) = [];
    end
end
F
x_f = F(:,1);
y_f = F(:,2);
k_a = boundary(x_a,y_a);
hold on;
%plot(x_a(k_a),y_a(k_a),'r');
k_b = boundary(x_b,y_b);
hold on;
%plot(x_b(k_b),y_b(k_b),'g');
k_f = boundary(x_f,y_f);
hold on;
% minkow = plot(x_f(k_f),y_f(k_f));
