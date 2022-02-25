%We are going to use a x-y space and a triple for loop. It's going to run a
%bit slow but the basic idea is we look at a certain x, y position, find
%the thetas corresponding to that, and then generate the forces in each
%direction.
clear; close all; clc
NX = 15;
NY = 15;
tau1 = [-1, -1, 1, 1]*0.536;
tau2 = [-1, 1, -1, 1]*0.536;

a = 0.1;
b = 0.5;

syms t1 t2
X = b*cos(t1)+b*cos(pi+t2);
Y = b*sin(t1)+b*sin(pi+t2);

xspace = linspace(0, 1, NX);
yspace = linspace(-0.5, 0.5, NY);

%Do jacobian, convert to matlabFunction because symbolic math is very very
%slow
J = [diff(X, t1), diff(X, t2); diff(Y, t1), diff(Y, t2)];

J11 = matlabFunction(J(1, 1));
J12 = matlabFunction(J(1, 2));
J21 = matlabFunction(J(2, 1));
J22 = matlabFunction(J(2, 2));

F_tablex1 = zeros(NX, NY);
F_tabley1 = zeros(NX, NY);
F_tablex2 = zeros(NX, NY);
F_tabley2 = zeros(NX, NY);
F_table = zeros(NX, NY);
progress = 1;
for i = 1:NX
    for j = 1:NY
        eqn1 = X == xspace(i);
        eqn2 = Y == yspace(j);

        thetasols = solve([eqn1, eqn2], [t1, t2]);

        theta1 = thetasols.t1;
        theta2 = thetasols.t2;
        
        numsols = length(theta1);
        min_x = Inf;
        min_y = Inf;
        max_x = -Inf;
        max_y = -Inf;
        max_f = -Inf;

        for l = 1:numsols
            theta1_curr = double(theta1(l));
            theta2_curr = double(theta2(l));

            %fix angle
            while theta1_curr < -pi
                theta1_curr = theta1_curr + 2*pi;
            end
            while theta1_curr > pi
                theta1_curr = theta1_curr - 2*pi;
            end
            while theta2_curr < -pi
                theta2_curr = theta2_curr + 2*pi;
            end
            while theta2_curr > pi
                theta2_curr = theta2_curr - 2*pi;
            end
            %invalid cases
            if ((~isreal([theta1_curr, theta2_curr])) || (theta2_curr - 35*pi/180 < theta1_curr || theta2_curr > pi - 35*pi/180 + theta1_curr)) 

%             if (~isreal([theta1_curr, theta2_curr]) && ...
%                 ((theta1_curr <= 0 && theta1_curr >= -pi/2) && (theta2_curr <= pi && theta2_curr >= -pi/2))) ...
%                 || (theta2_curr - 35*pi/180 < theta1_curr || theta2_curr > pi - 35*pi/180 + theta1_curr) 

                continue;
            end
    
            tau = [0; 0;];
            if i == 1 && j == 1
                fprintf("My current x, y is: (%.3f %.3f)", xspace(i), yspace(j))
                fprintf("According to my X, Y equations, my x, y is: (%.3f, %.3f)", subs(X,[t1, t2], [theta1_curr, theta2_curr]), subs(Y,[t1, t2], [theta1_curr, theta2_curr]));
            end

            for k = 1:4
                tau(1) = tau1(k);
                tau(2) = tau2(k);
                J_curr = [J11(theta1_curr), J12(theta2_curr); J21(theta1_curr), J22(theta2_curr)]';
                F_curr = pinv(J_curr, 1e-4) * tau * 10;
                

                if F_curr(1) > max_x
                    max_x = F_curr(1);
                end
                if F_curr(2) > max_y
                    max_y = F_curr(2);
                end
                if norm(F_curr) > max_f
                    max_f = norm(F_curr);
                end
                if F_curr(1) < min_x
                    min_x = F_curr(1);
                end
                if F_curr(2) < min_y
                    min_y = F_curr(2);
                end
            end
        end
        F_tablex1(i, j) = max_x;
        F_tabley1(i, j) = max_y;
        F_tablex2(i, j) = min_x;
        F_tabley2(i, j) = min_y;
        F_table(i, j) = max_f;
    end
    fprintf("Finished %d/%d progress\n", progress, NX);
    progress = progress + 1;
end
F_tablex1 = real(F_tablex1);
F_tablex2 = real(F_tablex2);
F_tabley1 = real(F_tabley1);
F_tabley2 = real(F_tabley2);
F_table = real(F_table);

writematrix(F_tablex1,'Fx1.csv','Delimiter','tab')
writematrix(F_tablex2,'Fx2.csv','Delimiter','tab')
writematrix(F_tabley1,'Fy1.csv','Delimiter','tab')
writematrix(F_tabley2,'Fy2.csv','Delimiter','tab')
writematrix(F_table,'F.csv','Delimiter','tab')

%% Plotting
[X, Y] = meshgrid(xspace, yspace);
figure; hold on;
subplot(2, 1, 1);
surf(X, Y, F_tablex1)
colorbar;
shading("interp");
graph_name = sprintf("Max Force (x) in positive direction");
title(graph_name, "Interpreter", "latex");
xlabel("\theta_1")
ylabel("\theta_2")
h = rotate3d;
h.RotateStyle = 'box';
h.Enable = 'on';

subplot(2, 1, 2);
surf(X, Y, F_tablex2)
graph_name = sprintf("Max Force (x) in negative direction");
title(graph_name, "Interpreter", "latex");
xlabel("X")
ylabel("Y")
colorbar;
shading("interp");
h = rotate3d;
h.RotateStyle = 'box';
h.Enable = 'on';


figure;
subplot(2, 1, 1);
surf(X, Y, F_tabley1)
colorbar;
shading("interp");
graph_name = sprintf("Max Force (y) in positive direction");
title(graph_name, "Interpreter", "latex");
xlabel("\theta_1")
ylabel("\theta_2")
h = rotate3d;
h.RotateStyle = 'box';
h.Enable = 'on';

subplot(2, 1, 2);
surf(X, Y, F_tabley2)
graph_name = sprintf("Max Force (y) in negative direction");
title(graph_name, "Interpreter", "latex");
xlabel("X")
ylabel("Y")
colorbar;
shading("interp");
h = rotate3d;
h.RotateStyle = 'box';
h.Enable = 'on';

figure;
surf(X, Y, F_table)
colorbar;
shading("interp");
graph_name = sprintf("Max Force (norm)");
title(graph_name, "Interpreter", "latex");
xlabel("X")
ylabel("Y")
colorbar;
shading("interp");
h = rotate3d;
h.RotateStyle = 'box';
h.Enable = 'on';

