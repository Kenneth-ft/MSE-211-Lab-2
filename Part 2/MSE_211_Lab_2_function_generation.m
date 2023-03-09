function MSE_211_Lab_2_function_generation
close all; clear all;
% This is the main file where the inputs and type of problem are defined

global inputs 

%Define Inputs (Position of r2 in first configuration)
inputs.R2x = 1;
inputs.R2y = 1;

%Define the input angles (in radians)
inputs.beta(1) = 60*(pi/180);   %In radians
inputs.beta(2) = 180*(pi/180);   %In radians
inputs.beta(3) = 270*(pi/180);  %In radians

%Define the output angles
inputs.gamma(1) = 90*(pi/180);	%In radians
inputs.gamma(2) = 120*(pi/180);	%In radians
inputs.gamma(3) = 210*(pi/180);	%In radians

function_generation %Call motion generation function (see below)
end

function function_generation

%Optimization Left Side
%Initial values (INPUT INITIAL ESTIMATES)
% r3x = -2; 
% r3y= 4;
% r4x = -3;
% r4y = 5;
% phi(1)= 20*(pi/180);    %In radians
% phi(2)= 30*(pi/180);    %In radians
% phi(3)= 40*(pi/180);    %In radians

r3x = 0.5; 
r3y= -0.5;
r4x = 1; 
r4y = -0.5;
phi(1)=-10*pi/180;
phi(2)=-70*pi/180;
phi(3)=-167*pi/180;

% Initial esimates in vector form (DO NOT CHANGE)
x0=[r3x,r3y,r4x,r4y,phi]';

%Call Optimization Algorithm (DO NOT CHANGE)
options = optimoptions(@fminunc,'Algorithm','quasi-newton');
[x,Fval]= fminunc(@coupled_sides,x0,options); %Call optimization algorithm

%Call animation (DO NOT CHANGE)
animation(x,Fval)
end

function F=coupled_sides(x)
global inputs

%Redefine your variables (DO NOT CHANGE)
r2x=inputs.R2x;
r2y=inputs.R2y;
r3x=x(1);
r3y=x(2);
r4x=x(3);
r4y=x(4);
phi(1)=x(5);
phi(2)=x(6);
phi(3)=x(7);

%THE RELATIVE EQUATIONS FOR EACH PRECISION POINT
for i = 1:1:3
    fx(i) = - r2x*(cos(inputs.beta(i))-1) + r2y*sin(inputs.beta(i)) - r3x*(cos(phi(i))-1) + r3y*sin(phi(i)) + r4x*(cos(inputs.gamma(i))-1) - r4y*sin(inputs.gamma(i));
    fy(i) = - r2y*(cos(inputs.beta(i))-1) - r2x*sin(inputs.beta(i)) - r3y*(cos(phi(i))-1) - r3x*sin(phi(i)) + r4y*(cos(inputs.gamma(i))-1) + r4x*sin(inputs.gamma(i));
end
%WRITE YOUR OBJECTIVE FUNCTION 
    F = sqrt(fx(1).^2 + fx(2).^2 + fx(3).^2 + fy(1).^2 + fy(2).^2 + fy(3).^2)
end

function animation(x,Fval)
global inputs 
%In this function you might not have to change anything for the lab 
%(assuming that you obtain a similar solution to mine). However, you will
%probably have to make changes for the project, depending on your outputs.
%THE SECTIONS TO BE CHANGED ARE HIGHLIGHTED WITH LINES / CAPITAL LETTERS

% Set up the screen size 
set(0,'Units','pixels');   % Set root units to pixels
dim = get(0,'ScreenSize'); % Get screen size
figure('doublebuffer','on','Position',[0,20,dim(3),dim(4)-100],...  %... line continues in the next row
    'Name','3D Object','NumberTitle','off'); %Create a screen size figure
set(gcf,'color', [1 1 1]) %Background Colour
axis equal; hold all;


% Redefining the variables of the Left side optimization outputs 
R2=[inputs.R2x, inputs.R2y]; 
R3=[x(1), x(2)];
R4=[x(3), x(4)];
phi(1)=x(5); phi(2)=x(6); phi(3)=x(7);

%Finding the length of the links for the left side
r2=norm(R2); r3=norm(R3); r4=norm(R4); 

%Location of Ground Pivots
GP2 = [0,0]; 
GP4 = R2+R3-R4; 

% Finding the length of the frame 
r1=norm(GP2-GP4);

% Defining angles to be between -pi and pi
for i=1:3
if phi(i)>pi; phi(i)=phi(i)-2*pi; end
if phi(i)<-pi; phi(i)=phi(i)+2*pi; end
end

% Output script of the solution obtained with the optimization
disp(' ')
fprintf('--------------------------------------------------------------------------------------------------------------\n');
disp('      Fval            r1            r2              r3             r4           phi1          phi2          phi3')
fprintf('--------------------------------------------------------------------------------------------------------------\n');
ffa=[Fval   r1 r2 r3 r4 phi(1)*180/pi phi(2)*180/pi phi(3)*180/pi];
fprintf('%14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f\n',ffa');

% Obtaining input and output angles (theta2 and theta4)

theta2_0= atan2(R2(2),R2(1));
theta2_f=theta2_0+inputs.beta(3);
theta4_0= atan2(R4(2),R4(1));
theta4_f=theta4_0+inputs.gamma(3);

%Define frame angle (always the same)
theta_1=atan2((GP4(2)-GP2(2)),(GP4(1)-GP2(1)));

%Ploting
%Plot frame link
line([GP2(1),GP4(1)],[GP2(2),GP4(2)], 'LineWidth', 3, 'Color', 'black')
plot(GP2(1),GP2(2), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k')
plot(GP4(1),GP4(2), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k')

%Plot r2 in all four configurations
plot([GP2(1);GP2(1)+r2*cos(theta2_0)],[GP2(2);GP2(2)+r2*sin(theta2_0)],'b'); 
plot([GP2(1);GP2(1)+r2*cos(theta2_0+inputs.beta(1))],[GP2(2);GP2(2)+r2*sin(theta2_0+inputs.beta(1))],'b');
plot([GP2(1);GP2(1)+r2*cos(theta2_0+inputs.beta(2))],[GP2(2);GP2(2)+r2*sin(theta2_0+inputs.beta(2))],'b');
plot([GP2(1);GP2(1)+r2*cos(theta2_0+inputs.beta(3))],[GP2(2);GP2(2)+r2*sin(theta2_0+inputs.beta(3))],'b');

%Plot r4 in all four configurations
plot([GP4(1);GP4(1)+r4*cos(theta4_0)],[GP4(2);GP4(2)+r4*sin(theta4_0)],'r'); 
plot([GP4(1);GP4(1)+r4*cos(theta4_0+inputs.gamma(1))],[GP4(2);GP4(2)+r4*sin(theta4_0+inputs.gamma(1))],'r');
plot([GP4(1);GP4(1)+r4*cos(theta4_0+inputs.gamma(2))],[GP4(2);GP4(2)+r4*sin(theta4_0+inputs.gamma(2))],'r');
plot([GP4(1);GP4(1)+r4*cos(theta4_0+inputs.gamma(3))],[GP4(2);GP4(2)+r4*sin(theta4_0+inputs.gamma(3))],'r');

%Plot r3 in all four configurations
plot([GP2(1)+r2*cos(theta2_0);GP4(1)+r4*cos(theta4_0)],[GP2(2)+r2*sin(theta2_0);GP4(2)+r4*sin(theta4_0)],'g')
plot([GP2(1)+r2*cos(theta2_0+inputs.beta(1));GP4(1)+r4*cos(theta4_0+inputs.gamma(1))],[GP2(2)+r2*sin(theta2_0+inputs.beta(1));GP4(2)+r4*sin(theta4_0+inputs.gamma(1))],'g')
plot([GP2(1)+r2*cos(theta2_0+inputs.beta(2));GP4(1)+r4*cos(theta4_0+inputs.gamma(2))],[GP2(2)+r2*sin(theta2_0+inputs.beta(2));GP4(2)+r4*sin(theta4_0+inputs.gamma(2))],'g')
plot([GP2(1)+r2*cos(theta2_0+inputs.beta(3));GP4(1)+r4*cos(theta4_0+inputs.gamma(3))],[GP2(2)+r2*sin(theta2_0+inputs.beta(3));GP4(2)+r4*sin(theta4_0+inputs.gamma(3))],'g')

%__________________________________________________________________________
%Definine range of theta2 (MAKE SURE IT WILL ROTATE IN THE RIGHT DIRECTION)
%The mechanism can rotate clockwise or counterclockwise, also the magnitude 
%of the angle is important for example if you move counterclockwise from 
%theta2_0=100 (deg) to theta2_f= 20 (deg), then you have to add 360 deg to 
%theta2_f, as shown below 
theta_2=+theta2_0:(theta2_f-theta2_0)/100:theta2_f;
%__________________________________________________________________________

% Pre-allocating memory 
Theta_3=NaN(length(theta_2),1); Theta_4=NaN(length(theta_2),1); 

%Animating Mechanism
for i=1:length(theta_2)
    %Displacement Analysis
    %(FOR THE PROJECT)  replace the section below with your displacement 
    %analysis code (shown below is the analytic solution) 
    %__________________________________________
    %Analytic Solution of Displacement Analysis
    a = r1*cos(theta_1) - r2*cos(theta_2(i));
    b = r1*sin(theta_1) - r2*sin(theta_2(i));
    c = (r3^2 - a^2 -r4^2 - b^2)/(2*r4);
    
    % Second solution is obtained by changing + with -, i.e. atan2(b,a) - atan
    theta_4 = atan2(b,a) - atan2(sqrt(a^2+b^2-c^2),c);
    theta_3 = atan2(b+r4*sin(theta_4),a+r4*cos(theta_4));
    %_________________________________________
    
    %Plotting Links
    R(2)=line([GP2(1),GP2(1)+r2*cos(theta_2(i))],[GP2(2),GP2(2)+r2*sin(theta_2(i))], 'LineWidth', 3);
    R(3)=line([GP2(1)+r2*cos(theta_2(i)), GP4(1)+r4*cos(theta_4)],[GP2(2)+r2*sin(theta_2(i)), GP4(2)+r4*sin(theta_4)], 'LineWidth', 2);   
    R(4)=line([GP4(1), GP4(1)+r4*cos(theta_4)],[GP4(2),GP4(2)+r4*sin(theta_4)], 'LineWidth', 3);
         
    %Plotting joints
    C(1)=plot(GP2(1)+r2*cos(theta_2(i)),GP2(2)+r2*sin(theta_2(i)), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k');
    C(2)=plot(GP4(1)+r4*cos(theta_4),GP4(2)+r4*sin(theta_4), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k');
    
    %_________________________________________
    % Scaling and pausing animation SCALE YOUR ANIMATION BASED ON YOUR WORKSPACE 
    axis equal; axis([-3 3 -2 2])
    pause(0.1)
    %_________________________________________

    % Turn off links and joints
    if i < length(theta_2)
        for ii=2:4; set(R(ii),'visible','off'); end
        set(C,'visible','off')
    end
    
    % Collect all Thetas (In case you want to plot them)
    Theta_3(i)=theta_3;
    Theta_4(i)=theta_4;
end

end
