
% Geometric control of Quadrotor on SE(3)
% http://www.math.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf
% 
% Hybrid Robotics Lab
% Carnegie Mellon University
% Author: vkotaru@andrew.cmu.edu
% Date: June-8-2016
% Last Updated: June-8-2016

%% INITIALZING WORKSPACE
% ======================
% Clear workspace
% ---------------
clear all; 
close all; 
clc;

% Add Paths
% ----------
% Adding path to 'Geometric Control Toolbox'
addpath('./Geometry-Toolbox/');


%% INITIALZING PARAMETERS
% ======================
% System constants and parameters
data.params.mQ = 0.5 ;
data.params.J = diag([0.557, 0.557, 1.05]*10e-2);
data.params.g = 9.81 ;
data.params.e1 = [1;0;0] ;
data.params.e2 = [0;1;0] ;
data.params.e3 = [0;0;1] ;


%% INTIALIZING - INTIAL CONDITIONS
% ================================
% Zero Position 
% -------------
xQ0 = [];
vQ0 = zeros(3,1);

R0 = RPYtoRot_ZXY(010*pi/180,0*pi/180, 0*pi/180) ;
Omega0 = zeros(3,1);

xQ0 = [1;3;2]; 1*ones(3,1);
vQ0 = zeros(3,1);
% 
R0 = RPYtoRot_ZXY(0*pi/180, 10*pi/180, 20*pi/180) ;
Omega0 = zeros(3,1);


% Zero Initial Error- Configuration
% ---------------------------------
%[trajd0] = get_nom_traj(data.params,get_flats(0));
xQ0 = [0;0;0];
vQ0 = zeros(3,1);

R0 = eye(3);
Omega0 = zeros(3,1);

% state - structure
% -----------------
% [xL; vL; R; Omega]
% setting up x0 (initial state)
% -----------------------------
x0 = [xQ0; vQ0; reshape(R0,9,1); Omega0 ];

%% SIMULATION
% ==========
disp('Simulating...') ;
odeopts = odeset('RelTol', 1e-8, 'AbsTol', 1e-9) ;
% odeopts = [] ;
[t, x] = ode15s(@odefun_quadDynamics, [0 15], x0, odeopts, data) ;

% Computing Various Quantities
disp('Computing...') ;
ind = round(linspace(1, length(t), round(1*length(t)))) ;
% ind = 0:length(t);
for i = ind
   [~,xd_,f_,M_] =  odefun_quadDynamics(t(i),x(i,:)',data);
   xd(i,:) = xd_';
   psi_exL(i) = norm(x(i,1:3)-xd(i,1:3));
   psi_evL(i) = norm(x(i,4:6)-xd(i,4:6));
   
   Rd = reshape(xd(i,7:15),3,3); R = reshape(x(i,7:15),3,3);  
   PsiR(i) = 0.5*trace(eye(3)-Rd'*R);
   f(i,1)= f_;
   M(i,:)= M_';
end


%% PLOTS
% =====
fig_1 = figure;
subplot(2,2,1);
plot(t(ind),x(ind,1),'-g',t(ind),xd(ind,1),':r');
grid on; title('x');legend('x','x_d');%axis equal;
xlabel('time');ylabel('x [m]');
subplot(2,2,2);
plot(t(ind),x(ind,2),'-g',t(ind),xd(ind,2),':r');
grid on; title('y');legend('y','y_d');%axis equal;
xlabel('time');ylabel('y [m]');
subplot(2,2,3);
plot(t(ind),x(ind,3),'-g',t(ind),xd(ind,3),':r');
grid on; title('z');legend('z','z_d');%axis equal;
xlabel('time');ylabel('z [m]');
subplot(2,2,4);
plot3(x(ind,1),x(ind,2),x(ind,3),'-g',xd(ind,1),xd(ind,2),xd(ind,3),':r');
grid on; title('trajectory');legend('traj','traj_d');%axis equal;
xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
sgtitle({'Geometric Tracking Control of a Quadrotor UA V on SE(3) ' , 'Taeyoung Lee, Melvin Leok, and N. Harris McClamroch'});

% figure;
% subplot(3,1,1);
% plot(t(ind),psi_exL(ind));
% grid on; title('position error');legend('psi-exL');
% subplot(3,1,2);
% plot(t(ind),psi_evL(ind));
% grid on; title('velocity error');legend('psi-evL');
% subplot(3,1,3);
% plot(t(ind), PsiR(ind));
% grid on; title('$$\Psi_R$$');

if ismac
    % Code to run on Mac platform
elseif isunix
    % Code to run on Linux platform
elseif ispc
    fig_1.WindowState = 'maximized';
    Image = getframe(fig_1);
    imwrite(Image.cdata, './figures/quad.jpg');
else
end

% % ANIMATION
% % =========
% keyboard;
animate_3dquad(t(ind), x(ind,:));




