function [trans_pts] = call_trans(Poly1, Poly2, F)

mx_1 = mean(Poly1(:,1));
my_1 = mean(Poly1(:,2));

mx_2 = mean(Poly2(:,1));
my_2 = mean(Poly2(:,2));

mx_f = mean(F(1:end-1,1));
my_f = mean(F(1:end-1,2));

dx = mx_1 - mx_f; % set translation values
dy = my_1 - my_f;

% create a simple box to draw and manipulate
% row 1 is x coords row 2 is y coords
F_t = F';
[n m] = size(F_t);

% create a 2D Translation in dx, dy NEED TO MAKE A HOMOGENEOUS COORDS matrix
trans = [1 0 dx;0 1 dy; 0 0 1];

% Do Translation
% Make PTS HOMOGENEOUS
homogeneous_pts = [F_t; ones(1,9)];

%Translate
trans_pts = trans*homogeneous_pts;



  