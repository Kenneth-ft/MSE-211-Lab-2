clear;
close all;
tol = 10^-4;
% Define call function numerically
[x1,y1] = meshgrid(-5:0.1:5,-5:0.1:5);
f = Lab_2_Fun(x1,y1);
%z = meshgrid(f);


% Initial guess
x0 = [0;0]; %point 1
% x0 = [-1;0]; %point 2
% x0 = [1;-5]; % point 3
% x0 = [-5;0]; % point 4

% Call methods
%1.1
[X,traj,Z,k,Err] = Lab_2_sdm(x0,tol);
%1.2
%[X,traj,Z,k,Err] = Lab_2_Newton(x0,tol);
%1.3
% options = optimoptions('fminunc','GradObj','on','Algorithm','trust-region');
% [X,FVAL,EXITFLAG,OUTPUT] = fminunc(@Lab_2_Funn,x0)
% X
% FVAL


% Plot surface
surface = figure; figure(surface);
surf(x1,y1,f); shading interp;
hold on
%Add trajectory for finding minima (3D plot)
plot3(traj(1,:), traj(2,:), Z(1,:) ,'-k+')

% Plot contour (2D plot)
contour_graph = figure; figure(contour_graph);
contour(x1,y1,f,100); 
hold on;
plot(X(1,:), X(2,:), '-k+');

%final point
X
[~,index] = min(Z);
%function evaluation
foptimum = Z(index)
ERRAprox = Err(index)
iterations = k

trajt = transpose(traj);
Zt = transpose(Z);
eat = transpose(Err);

