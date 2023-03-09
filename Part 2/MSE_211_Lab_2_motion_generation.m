function MSE_211_Lab_2_motion_generation
close all; clear all;
% This is the main file where the inputs and type of problem are defined

global inputs 

%Define the Precision Points (Position and Orientation)
%Position Vectors Px and Py (INPUT POSITIONS)
inputs.Px(1) = 5;
inputs.Py(1) = 1;
inputs.Px(2) = 7;
inputs.Py(2) = 7;
inputs.Px(3) = 10;
inputs.Py(3) = 9;
inputs.Px(4) = 12;
inputs.Py(4) = 8; 

%Define Orientation in radians (INPUT ANGLES)
inputs.ang(1) = 0; %In radians
inputs.ang(2) = 60*(pi/180);   %In radians
inputs.ang(3) = 30*(pi/180);  %In radians
inputs.ang(4) = 0;   %In radians

% Desired angles with respect to the first angle (DO NOT CHANGE)
inputs.alpha(1)=inputs.ang(2)-inputs.ang(1);
inputs.alpha(2)=inputs.ang(3)-inputs.ang(1);
inputs.alpha(3)=inputs.ang(4)-inputs.ang(1);

% Desired positions with respect to the first positions (DO NOT CHANGE)
inputs.deltax(1)=inputs.Px(2)-inputs.Px(1);
inputs.deltay(1)=inputs.Py(2)-inputs.Py(1);

inputs.deltax(2)=inputs.Px(3)-inputs.Px(1);
inputs.deltay(2)=inputs.Py(3)-inputs.Py(1);

inputs.deltax(3)=inputs.Px(4)-inputs.Px(1);
inputs.deltay(3)=inputs.Py(4)-inputs.Py(1);

motion_generation %Call motion generation function (see below)
end

function motion_generation

%Optimization Left Side
%Initial values (INPUT INITIAL ESTIMATES)
r2x = -3; 
r2y= 1;
r3ax = 2; 
r3ay = 4;
beta(1)=-30*pi/180;
beta(2)=-100*pi/180;
beta(3)=-250*pi/180;

% Initial esimates in vector form (DO NOT CHANGE)
x0=[r2x,r2y,r3ax,r3ay,beta]';

%Call Optimization Algorithm (DO NOT CHANGE)
options = optimoptions(@fminunc,'Algorithm','quasi-newton');
[Lx,LFval]= fminunc(@left_side,x0,options); %Call optimization algorithm

%Optimization Right side
%Initial values (INPUT INITIAL ESTIMATES)
r4x = -3;
r4y = 3;
r3bx = -2;
r3by= 4;
gamma(1)=-40*pi/180;
gamma(2)=-90*pi/180;
gamma(3)=-200*pi/180;

% Initial esimates in vector form (DO NOT CHANGE)
x0=[r4x,r4y,r3bx,r3by,gamma]';

%Call Optimization Algorithm (DO NOT CHANGE)
[Rx,RFval]= fminunc(@right_side,x0,options);%Call optimization algorithm

%Call animation (DO NOT CHANGE)
animation(Lx,Rx,LFval,RFval)
end

function F=left_side(x)
global inputs

%Redefine your variables (DO NOT CHANGE)
r2x=x(1);
r2y=x(2);
r3ax=x(3);
r3ay=x(4);
beta(1)=x(5);
beta(2)=x(6);
beta(3)=x(7);

%THE RELATIVE EQUATIONS FOR EACH PRECISION POINT
for i = 1:1:3
    fx(i) = r2x + r3ax + inputs.deltax(i) - r3ax*cos(inputs.alpha(i)) + r3ay*sin(inputs.alpha(i)) - r2x*cos(beta(i)) + r2y*sin(beta(i));
    fy(i) = r2y + r3ay + inputs.deltay(i) - r3ay*cos(inputs.alpha(i)) - r3ax*sin(inputs.alpha(i)) - r2y*cos(beta(i)) - r2x*sin(beta(i));
end
%YOUR OBJECTIVE FUNCTION 
F = sqrt(fx(1).^2 + fx(2).^2 + fx(3).^2 + fy(1).^2 + fy(2).^2 + fy(3).^2);

end

function G=right_side(x)
global inputs

%Redefine your variables (DO NOT CHANGE)
r4x=x(1);
r4y=x(2);
r3bx=x(3);
r3by=x(4);
gamma(1)=x(5);
gamma(2)=x(6);
gamma(3)=x(7);


for i = 1:1:3
    %WRITE THE RELATIVE EQUATIONS FOR EACH PRECISION POINT
    gx(i) = r4x + r3bx + inputs.deltax(i) - r3bx*cos(inputs.alpha(i)) + r3by*sin(inputs.alpha(i)) - r4x*cos(gamma(i)) + r4y*sin(gamma(i));
    gy(i) = r4y + r3by + inputs.deltay(i) - r3by*cos(inputs.alpha(i)) - r3bx*sin(inputs.alpha(i)) - r4y*cos(gamma(i)) - r4x*sin(gamma(i));
end

%YOUR OBJECTIVE FUNCTION 
G = sqrt(gx(1).^2 + gx(2).^2 + gx(3).^2 + gy(1).^2 + gy(2).^2 + gy(3).^2);

end

function animation(Lx,Rx,LFval,RFval)
global inputs 
%In this function you might not have to change anything for the lab 
%(assuming that you obtain a similar solution to mine). However, you will
%probably have to make changes for the project, depending on your outputs.
%THE SECTIONS TO BE CHANGED ARE HIGHLIGHTED WITH LINES / CAPITAL LETTERS

% Precision Points as a vector
P1=[inputs.Px(1),inputs.Py(1)];
P2=[inputs.Px(2),inputs.Py(2)];
P3=[inputs.Px(3),inputs.Py(3)];
P4=[inputs.Px(4),inputs.Py(4)];

% Set up the screen size 
set(0,'Units','pixels');   % Set root units to pixels
dim = get(0,'ScreenSize'); % Get screen size
figure('doublebuffer','on','Position',[0,20,dim(3),dim(4)-100],...  %... line continues in the next row
    'Name','3D Object','NumberTitle','off'); %Create a screen size figure
set(gcf,'color', [1 1 1]) %Background Colour

%______________________________________________________________________
%Drawing the box (THIS PART IS ONLY FOR THE LAB)
box_shape = [0,0; 2,0; 2,2; 0,2; 0,0]; %Creating a 2x2 box
box=NaN(5,2,4); %pre-allocating memory
for i=1:4 % rotating and tranlating the box
box(:,:,i)=[cos(inputs.ang(i))*box_shape(:,1)-sin(inputs.ang(i))*box_shape(:,2),sin(inputs.ang(i))*box_shape(:,1)+cos(inputs.ang(i))*box_shape(:,2)] + repmat([inputs.Px(i), inputs.Py(i)],5,1);
line(box(:,1,i),box(:,2,i)); %ploting the box
end
axis([-5 15 -5 15]); axis equal; hold all;  %CHANGE AXIS SCALE TO FIT YOUR PROBLEM
%______________________________________________________________________

% Redefining the variables of the Left side optimization outputs 
R2=[Lx(1),Lx(2)]; R3a=[Lx(3),Lx(4)]; 
beta(1)=Lx(5); beta(2)=Lx(6); beta(3)=Lx(7);

%Finding the length of the links for the left side
r2=norm(R2); r3a=norm(R3a);

% Redefining the variables of the Right Side optimization outputs 
R4=[Rx(1),Rx(2)]; R3b=[Rx(3),Rx(4)];
gamma(1)=Rx(5); gamma(2)=Rx(6); gamma(3)=Rx(7);

%Finding the length of the links for the right side
r4=norm(R4); r3b=norm(R3b);

%Location of Ground Pivots
GP2 = P1-R2-R3a;
GP4 = P1-R4-R3b; 

% Finding the length of the frame and couple links
r1=norm(GP2-GP4);
r3 = norm((R3a-P1)-(R3b-P1)); 

% Defining output angles to be between -pi and pi
for i=1:3
if beta(i)>pi; beta(i)=beta(i)-2*pi; end
if beta(i)<-pi; beta(i)=beta(i)+2*pi; end
if gamma(i)>pi; gamma(i)=gamma(i)-2*pi; end
if gamma(i)<pi; gamma(i)=gamma(i)+2*pi; end
end

% Output script of the solution obtained with the optimization
disp(' ')
fprintf('--------------------------------------------------------------------------------------------------------------\n');
disp(['      LFval           RFval            r1            r2              r3             r4           beta1          beta2          beta3         gamma1       gamma2       gamma3'])
fprintf('--------------------------------------------------------------------------------------------------------------\n');
ffa=[LFval    RFval   r1 r2 r3 r4 beta(1)*180/pi beta(2)*180/pi beta(3)*180/pi gamma(1)*180/pi gamma(2)*180/pi gamma(3)*180/pi];
fprintf('%14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f %14.8f\n',ffa');

% Obtaining input and output angles (theta2 and theta4)
theta2_0= atan2(R2(2),R2(1));
theta2_f=theta2_0+beta(3);
theta4_0= atan2(R4(2),R4(1));
theta4_f=theta4_0+gamma(3);

%Define frame angle (always the same)
theta_1=atan2((GP4(2)-GP2(2)),(GP4(1)-GP2(1)));

%Ploting
%Plot frame link
line([GP2(1),GP4(1)],[GP2(2),GP4(2)], 'LineWidth', 3, 'Color', 'black')
plot(GP2(1),GP2(2), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k')
plot(GP4(1),GP4(2), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k')

%Plot r2 in all four configurations
plot([GP2(1);GP2(1)+r2*cos(theta2_0)],[GP2(2);GP2(2)+r2*sin(theta2_0)],'b'); 
plot([GP2(1);GP2(1)+r2*cos(theta2_0+beta(1))],[GP2(2);GP2(2)+r2*sin(theta2_0+beta(1))],'b');
plot([GP2(1);GP2(1)+r2*cos(theta2_0+beta(2))],[GP2(2);GP2(2)+r2*sin(theta2_0+beta(2))],'b');
plot([GP2(1);GP2(1)+r2*cos(theta2_0+beta(3))],[GP2(2);GP2(2)+r2*sin(theta2_0+beta(3))],'b');

%Plot r4 in all four configurations
plot([GP4(1);GP4(1)+r4*cos(theta4_0)],[GP4(2);GP4(2)+r4*sin(theta4_0)],'r'); 
plot([GP4(1);GP4(1)+r4*cos(theta4_0+gamma(1))],[GP4(2);GP4(2)+r4*sin(theta4_0+gamma(1))],'r');
plot([GP4(1);GP4(1)+r4*cos(theta4_0+gamma(2))],[GP4(2);GP4(2)+r4*sin(theta4_0+gamma(2))],'r');
plot([GP4(1);GP4(1)+r4*cos(theta4_0+gamma(3))],[GP4(2);GP4(2)+r4*sin(theta4_0+gamma(3))],'r');

%Plot r3 in all four configurations
plot([GP2(1)+r2*cos(theta2_0);GP4(1)+r4*cos(theta4_0);P1(1);GP2(1)+r2*cos(theta2_0)],[GP2(2)+r2*sin(theta2_0);GP4(2)+r4*sin(theta4_0);P1(2);GP2(2)+r2*sin(theta2_0)],'g')
plot([GP2(1)+r2*cos(theta2_0+beta(1));GP4(1)+r4*cos(theta4_0+gamma(1));P2(1);GP2(1)+r2*cos(theta2_0+beta(1))],[GP2(2)+r2*sin(theta2_0+beta(1));GP4(2)+r4*sin(theta4_0+gamma(1));P2(2);GP2(2)+r2*sin(theta2_0+beta(1))],'g')
plot([GP2(1)+r2*cos(theta2_0+beta(2));GP4(1)+r4*cos(theta4_0+gamma(2));P3(1);GP2(1)+r2*cos(theta2_0+beta(2))],[GP2(2)+r2*sin(theta2_0+beta(2));GP4(2)+r4*sin(theta4_0+gamma(2));P3(2);GP2(2)+r2*sin(theta2_0+beta(2))],'g')
plot([GP2(1)+r2*cos(theta2_0+beta(3));GP4(1)+r4*cos(theta4_0+gamma(3));P4(1);GP2(1)+r2*cos(theta2_0+beta(3))],[GP2(2)+r2*sin(theta2_0+beta(3));GP4(2)+r4*sin(theta4_0+gamma(3));P4(2);GP2(2)+r2*sin(theta2_0+beta(3))],'g')

%__________________________________________________________________________
%Definine range of theta2 (MAKE SURE IT WILL ROTATE IN THE RIGHT DIRECTION)
%The mechanism can rotate clockwise or counterclockwise, also the magnitude 
%of the angle is important for example if you move counterclockwise from 
%theta2_0=100 (deg) to theta2_f= 20 (deg), then you have to add 360 deg to 
%theta2_f, as shown below 
theta_2=theta2_0:(2*pi+theta2_f-theta2_0)/100:2*pi+theta2_f;
%__________________________________________________________________________

%__________________________________________________________________________
%If the coupler plate is inverted in your animation, ADD A NEGATIVE SIGN TO
%THE phi angle, i.e., phi=-acos(...) 
%Defining coupler plate
R3=R3a-R3b;
phi=acos(dot([R3a(1) R3a(2)],[R3(1) R3(2)])/(r3*r3a));
r3a=norm(R3a);
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
    theta_4 = atan2(b,a) + atan2(sqrt(a^2+b^2-c^2),c);
    theta_3 = atan2(b+r4*sin(theta_4),a+r4*cos(theta_4));
    %_________________________________________
    
    %Plotting Links
    R(2)=line([GP2(1),GP2(1)+r2*cos(theta_2(i))],[GP2(2),GP2(2)+r2*sin(theta_2(i))], 'LineWidth', 3);
    R(3)=line([GP2(1)+r2*cos(theta_2(i)), GP2(1)+r2*cos(theta_2(i))+r3a*cos(theta_3-phi), GP4(1)+r4*cos(theta_4),GP2(1)+r2*cos(theta_2(i))],[GP2(2)+r2*sin(theta_2(i)),GP2(2)+r2*sin(theta_2(i))+r3a*sin(theta_3-phi), GP4(2)+r4*sin(theta_4),GP2(2)+r2*sin(theta_2(i))], 'LineWidth', 2);
    R(4)=line([GP4(1), GP4(1)+r4*cos(theta_4)],[GP4(2),GP4(2)+r4*sin(theta_4)], 'LineWidth', 3);
    
    %Filling translucent Plate 
    Plate=fill([GP2(1)+r2*cos(theta_2(i)), GP2(1)+r2*cos(theta_2(i))+r3a*cos(theta_3-phi), GP4(1)+r4*cos(theta_4),GP2(1)+r2*cos(theta_2(i))],[GP2(2)+r2*sin(theta_2(i)),GP2(2)+r2*sin(theta_2(i))+r3a*sin(theta_3-phi), GP4(2)+r4*sin(theta_4),GP2(2)+r2*sin(theta_2(i))],'b');
    set(Plate,'facealpha',0.5)
    
    %Plotting joints
    C(1)=plot(GP2(1)+r2*cos(theta_2(i)),GP2(2)+r2*sin(theta_2(i)), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k');
    C(2)=plot(GP4(1)+r4*cos(theta_4),GP4(2)+r4*sin(theta_4), 'o', 'MarkerFaceColor', [0 0 1], 'MarkerSize',8, 'MarkerEdgeColor','k');
    
    %_________________________________________
    % Scaling and pausing animation SCALE YOUR ANIMATION BASED ON YOUR WORKSPACE 
    axis equal; axis([-5 15 -5 15])
    pause(0.1)
    %_________________________________________
    
    % Turn off links and joints
    if i < length(theta_2)
        for ii=2:4; set(R(ii),'visible','off'); end
        set(C,'visible','off')
        set(Plate,'visible','off')
    end
    
    % Collect all Thetas (In case you want to plot them)
    Theta_3(i)=theta_3;
    Theta_4(i)=theta_4;
end

end
