clear; close all; clc
NX = 200;
NY = 200;
%Max continuous motor torque is 536 mN*m
tau1 = [-1, -1, 1, 1]*0.536;
tau2 = [-1, 1, -1, 1]*0.536;

%we deemed a reasonable angle range to be around 0-90 for theta1, which is
%defined to be the angle of the first limb relative to a horizontal line,
%and more or less pi/20, 19pi/20 for theta2, the angle of the second limb
%relative to the first limb.
theta1_space = linspace(-pi/2, 0, NX);
theta2_space = linspace(-pi/2,pi, NY);
syms t1 t2
%initial design a = 0.1 b = 0.5
a = 0.1;
b = 0.5;

% X = b*(cos(t1)+sin(t1+t2-pi/2));
% Y = b*(sin(t1)+cos(t1+t2-pi/2));

%using Alex's X and Y coords for the e-e given a phantom arm configuration
% X = b*cos(t1)+b*cos(t1+pi-t2);
% Y = b*sin(t1)+b*sin(t1+pi-t2);

%Using X and Y coords for the e-e with angles that are independent of one
%another

X = b*cos(t1)+b*cos(180+t2);
Y = b*sin(t1)+b*sin(180+t2);
%Do jacobian, convert to matlabFunction because symbolic math is very very
%slow
J = [diff(X, t1), diff(X, t2); diff(Y, t1), diff(Y, t2)];

J11 = matlabFunction(J(1, 1));
J12 = matlabFunction(J(1, 2));
J21 = matlabFunction(J(2, 1));
J22 = matlabFunction(J(2, 2));

global_min_force = 10000;
global_min_coords = [-1, -1, -1];
tau = [0; 0;];
progress = 1;
%loop 4 times for the different max torque combos
for i = 1:4
    tau(1) = tau1(i);
    tau(2) = tau2(i);
    F_tablex = zeros(NX, NY);
    F_tabley = zeros(NX, NY);
    F_table = zeros(NX, NY);
    %Go through a nested for loop, and try combinations of angles (using
    %the max continuous motor torques) and find the resultant force
    for k = 1:NX
        for l = 1:NY
            t1 = theta1_space(k);
            t2 = theta2_space(l);

            J_curr = [J11(t1), J12(t2); J21(t1), J22(t2)]';
            %Multiply by 10 because of a 10:1 transmission? I'm still using
            %a pseudoinverse here because apparently we got a lot of
            %singular matrices when we do inv()
            F_curr = pinv(J_curr, 1e-4) * tau * 10;
            F_norm = norm(F_curr);

            %record values for plotting later
            F_tablex(k, l) = F_curr(1);
            F_tabley(k, l) = F_curr(2);
            F_table(k, l) = F_norm;

            %update our global min if we have a value less than our current
            %min
            if F_norm < global_min_force
                global_min_force = F_norm;
                global_min_coords = [i, k, l];
            end
        end
    end
    %plotting stuff
    for k = 1:NX
        for l = 1:NY
            if (theta2_space(l) < theta1_space(k) || theta2_space(l) > pi + theta1_space(k))
                F_tablex(k, l) = nan;
                F_tabley(k, l) = nan;
                F_table(k, l) = nan;
            end
        end
    end
    


    [X, Y] = meshgrid(theta1_space, theta2_space);
    figure; hold on;
    subplot(3, 1, 1);
    surf(X, Y, F_tablex)
    colorbar;
    shading("interp");
    graph_name = sprintf("Max Force (x) when tau1 = %.3f, tau2 = %.3f", tau(1), tau(2));
    title(graph_name, "Interpreter", "latex");
    xlabel("\theta_1")
    ylabel("\theta_2")
    h = rotate3d;
    h.RotateStyle = 'box';
    h.Enable = 'on';

    subplot(3, 1, 2);
    surf(X, Y, F_tabley)
    colorbar;
    shading("interp");
    graph_name = sprintf("Max Force (y) when tau1 = %.3f, tau2 = %.3f", tau(1), tau(2));
    title(graph_name, "Interpreter", "latex");
    xlabel("\theta_1")
    ylabel("\theta_2")
    h = rotate3d;
    h.RotateStyle = 'box';
    h.Enable = 'on';

    subplot(3, 1, 3);
    surf(X, Y, F_table)
    colorbar;
    shading("interp");
    graph_name = sprintf("Max Force when tau1 = %.3f, tau2 = %.3f", tau(1), tau(2));
    title(graph_name, "Interpreter", "latex");
    xlabel("\theta_1")
    ylabel("\theta_2")
    h = rotate3d;
    h.RotateStyle = 'box';
    h.Enable = 'on';

    figure;
    surf(X, Y, F_table)
    colorbar;
    shading("interp");
    graph_name = sprintf("Max Force when tau1 = %.3f, tau2 = %.3f", tau(1), tau(2));
    title(graph_name, "Interpreter", "latex");
    xlabel("\theta_1")
    ylabel("\theta_2")
    h = rotate3d;
    h.RotateStyle = 'box';
    h.Enable = 'on';
    view(0, 90)

    fprintf("Finished %d/4 progress\n", progress);
    progress = progress + 1;
end


fprintf("Determined the minimum force possible to be %.4f N\n", global_min_force);
fprintf("This was achieved at: tau1 = %.4f, tau2 = %.4f, theta1 = %.4f, theta2 = %.4f\n", tau1(global_min_coords(1)), tau2(global_min_coords(1)), theta1_space(global_min_coords(2))*180/pi, theta2_space(global_min_coords(3))*180/pi);