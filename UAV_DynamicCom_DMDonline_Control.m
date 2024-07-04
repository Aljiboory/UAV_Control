%*****************************************************************
%****************** TRAJECTORY TRACKING **************************
%**************** AERIAL MANIPULATOR ROBOT ***********************
%*****************************************************************

clc; clear all; close all; warning off % Initialization
f = 30;
ts = 1/f;       % Sampling time
tfin = 120;      % Simulation time
t = 0:ts:tfin;
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% Variables defined by the TRAJECTORY and desired VELOCITIES
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = TrajectoriesDMD(2,t);
%% GENERALIZED DESIRED SIGNALS
hd = [xd; yd; zd; psid];
hd_p = [xdp;ydp;zdp;psidp];

%% a) Initial positions of the UAV
xu(1) = 0; 
yu(1) = 0; 
zu(1) = 2; 
psi(1)= 0;
h=[xu(1);yu(1);zu(1);psi(1)];

%% A & B Matrix of the DMD Offline dynamic system (Continuous)
load("A_B_values.mat"); 

load("UAV_Parameters.mat");
chi_real = chi';

%% Values init for DMD ONLINE (Discrete)
load("G&P_DMDonline_values_init.mat");
Ae = G(:,1:4);
Be = G(:,5:end);

%% Real initial speed of the UAV
v = [0;0;0;0];
v_est = v(:,1);
sample = 0;
A_E_P = reshape(Ae,16,1)
B_E_P = reshape(Be,16,1) 

A_c = (Ae-eye(4))/ts;
B_c = Be/ts;


A_c_aux = (Ae-eye(4))/ts;
B_c_aux = Be/ts;

%% Windowed DMD Online Value
m=6;
%***********************************************************************
%************************** CONTROLLER *********************************
%***********************************************************************
disp('Start of the program')
F_externl = zeros(4, length(t));
t_aux = (t>=20 & t<=60) | (t>=80 & t<=100);

F_externl(1,t_aux) = -0.4;
F_externl(2,t_aux) = 0.5;
F_externl(3,t_aux) = -0.1;

%% Filter 
% Filter force design
kp_f = 1;
wn_f = sqrt(kp_f);
kv_f = 1.1*1*wn_f;

Filter = tf([1], [1 kv_f kp_f]);
Filter_d = c2d(Filter, ts);
[num1d_filter, den1d_filter] = tfdata(Filter_d,'v');

% Filter coefficients
A_filter = num1d_filter(2:end);
B_filter = den1d_filter(2:end);

% Filter signals
% Force Filter
ul_memory = zeros(length(den1d_filter(2:end)),1);
um_memory = zeros(length(den1d_filter(2:end)),1);
un_memory = zeros(length(den1d_filter(2:end)),1);
w_memory = zeros(length(den1d_filter(2:end)),1);

ul_filter =  zeros(length(num1d_filter(2:end)),1);
um_filter =  zeros(length(num1d_filter(2:end)),1);
un_filter =  zeros(length(num1d_filter(2:end)),1);
w_filter =  zeros(length(num1d_filter(2:end)),1);

for k=1:length(t)-1
tic
%% 1) CONTROL LAW
vc(:,k) = Vc_UAV(hd_p(:,k),hd(:,k),xu(k),yu(k),zu(k),psi(k)); 
ul(k)=vc(1,k); um(k)=vc(2,k); un(k)=vc(3,k); w(k)=vc(4,k);
if k==1
    ulp = ul(k)/ts; ump = um(k)/ts;  unp= un(k)/ts; wp= w(k)/ts;
else
    ulp = (ul(k)-ul(k-1))/ts; ump = (um(k)-um(k-1))/ts;
    unp = (un(k)-un(k-1))/ts; wp = (w(k)-w(k-1))/ts;
end
 vcp = [ulp;ump;unp;wp];
vcp = [0;0;0;0];
%% DYNAMIC COMPENSATION
vref(:,k) = dynamicComDMD_online(A_c_aux, B_c_aux, vcp, vc(:,k), v(:,k), 3, 4); %2 6
%vref(:,k) = vc(:,k);
%% 2) UAV DYNAMICS (SPEED AND POSITION)
%[v_real(:, k+1),Tu(:,k)] = dyn_model_adapUAV(chi_real, v_real(:,k), vref(:,k), psi(k), L, ts, k, F_extern(:, k));
[v(:, k+1),Tu(:,k)] = DMD_dymamic_system(A,B,v(:,k), vref(:,k),ts,F_externl(:, k));

% Numerical integration using Runge-Kutta method
 J = [cos(psi(k)) -sin(psi(k)) 0 0;
         sin(psi(k)) cos(psi(k)) 0 0;
         0 0 1 0;
         0 0 0 1];
h_p(:,k) = J*v(:,k+1);

h(:,k+1) = h(:,k)+ UAV_RK4(h(:,k),v(:,k+1),ts);
xu(k+1) = h(1,k+1);
yu(k+1) = h(2,k+1);
zu(k+1) = h(3,k+1);      
psi(k+1) = Angulo(h(4,k));

%% A and B Estimation DMD ONLINE
A_c = (Ae-eye(4))/ts;
B_c = Be/ts;
v_est(:, k+1) = Ae*v_est(:,k)+ Be*vc(:,k);
rho = 1;
if sample >= m
    [Ae,Be,P,G] = DMD_Online(m,v_est,vc,v,P,G,k,rho);
    sample = 0;   
end
sample = sample + 1;

%% 3) Machine time   
dt(k) = toc;
A_E(:,k+1) = reshape(A,16,1);
B_E(:,k+1) = reshape(B,16,1);  

A_E_P(:,k+1) = reshape(Ae,16,1);
B_E_P(:,k+1) = reshape(Be,16,1);  
end

save("MIL_test.mat","dt","h","h_p","hd","hd_p","t","v","vc","vcp","vref");
disp('End of calculations')

%*******************************************************************%
%**************TRAJECTORY TRACKING ANIMATION ***********************%
%% *****************************************************************%
disp('Animation RUN')

% 1) Animation frame parameters
figure(1)
axis equal
view(-15,15) % Viewing angle
cameratoolbar
title ("Simulation")

% 2) Configure scale and color of the UAV
Drone_Parameters(0.08);
H1 = Drone_Plot_3D(xu(1),yu(1),zu(1),0,0,psi(1));hold on

% c) Plot of the desired trajectory
plot3(xd,yd,zd,'--')
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 5) Simulation of the aerial manipulator's motion
for k=1:400:length(t)  
% a) Remove the previous drawings of the aerial manipulator
delete(H1);
H1 = Drone_Plot_3D(xu(k),yu(k),zu(k),0,0,psi(k)); hold on
% b) Plot the desired vs actual position in each frame
plot3(xu(1:k),yu(1:k),zu(1:k),'r')
hold on
plot3(xd(1:k),yd(1:k),zd(1:k),'-.b')

pause(0.1)
end
legend('Actual Trajectory','Desired Trajectory')
disp('END Simulation RUN')  

%%
%********************************************************************
%************************** GRAPHS **********************************
%% ******************************************************************

%% 2) Error calculations
figure(2)
hxe= xd - xu;
hye= yd - yu;
hze= zd - zu;
psie= Angulo(psid-psi);
plot(t(1,1:length(xd)),hxe,'r',"LineWidth",1.5)
hold on, grid on
plot(t(1,1:length(yd)),hye,'--b',"LineWidth",1.5)
plot(t(1,1:length(zd)),hze,'-.k',"LineWidth",1.5)
plot(t(1,1:length(psid)),psie,':c',"LineWidth",1.5)
legend("x_e","y_e","z_e","\psi_e")
%title ("Position Errors")
ylabel('Position Errors'); xlabel('Time (Sec.)');

%% 3) Desired vs actual positions of the aerial manipulator's end-effector
figure(3)

subplot(4,1,1)
plot(t(1,1:length(xd)),xd,'r',"LineWidth",1.5)
hold on
plot(t(1,1:length(xu)),xu,'--b',"LineWidth",1.5)
legend("Desired","Actual")
ylabel('X (m)'); %xlabel('Time (Sec.)');
grid on
%title ("Desired and actual positions of the UAV")

subplot(4,1,2)
plot(t(1,1:length(yd)),yd,'r',"LineWidth",1.5)
hold on 
plot(t(1,1:length(yu)),yu,'--b',"LineWidth",1.5)
grid on
%legend("y_d","h_y")
ylabel('Y (m)'); %xlabel('Time (Sec.)');

subplot(4,1,3)
plot(t(1,1:length(zd)),zd,'r',"LineWidth",1.5)
hold on
plot(t(1,1:length(zd)), zu,'--b',"LineWidth",1.5)
grid on
%legend("z_d","h_z")
ylabel('Z (m)'); %xlabel('Time (Sec.)');

subplot(4,1,4)
plot(t(1,1:length(psi)),Angulo(psid),'r',"LineWidth",1.5)
hold on
plot(t(1,1:length(psi)),psi,'--b',"LineWidth",1.5)
grid on
%legend("\psi_d","\psi")
ylabel('\psi (rad)'); xlabel('Time (Sec.)');

%% 3) Desired vs actual positions of the end effector of the aerial manipulator
figure(4)
plot(t(1,1:length(vc)),vc(1,1:end),'r',"LineWidth",1.5)
hold on
plot(t(1:length(v)),v(1,1:end),'--b',"LineWidth",1.5)
hold on
plot(t(1:length(vref)),vref(1,1:end),'-.k',"LineWidth",1.5)
hold on
plot(t(1:length(v_est)),v_est(1,1:end),':c',"LineWidth",1.5)
grid on
legend("ulc","ul","ul_{ref}","ul_{est}")
ylabel('X (m/s)'); xlabel('Time (Sec.)');
title ("Desired and actual positions of the UAV")

%%
figure(5)
plot(t(1:length(vc)),vc(2,1:end),'r',"LineWidth",1.5)
hold on
plot(t(1:length(v)),v(2,1:end),'--b',"LineWidth",1.5)
hold on
plot(t(1:length(vref)),vref(2,1:end),'-.k',"LineWidth",1.5)
hold on
plot(t(1:length(v_est)),v_est(2,1:end),':c',"LineWidth",1.5)
grid on
legend("umc","um","um_{ref}","um_{est}")
ylabel('y (m/s)'); xlabel('Time (Sec.)');

%%
figure(6)
plot(t(1:length(vc)),vc(3,1:end),'r',"LineWidth",1.5)
hold on
plot(t(1:length(v)),v(3,1:end),'--b',"LineWidth",1.5)
hold on
plot(t(1:length(vref)),vref(3,1:end),'-.k',"LineWidth",1.5)
hold on
plot(t(1:length(v_est)),v_est(3,1:end),':c',"LineWidth",1.5)
grid on
legend("unc","un","un_{ref}","un_{est}")
ylabel('z (m/s)'); xlabel('Time (Sec.)');

%%
figure(7)
plot(t(1:length(vc)),vc(4,1:end),'r',"LineWidth",1.5)
hold on
plot(t(1:length(v)),v(4,1:end),'--b',"LineWidth",1.5)
hold on
plot(t(1:length(vref)),vref(4,1:end),'-.k',"LineWidth",1.5)
hold on
plot(t(1:length(v_est)),v_est(4,1:end),':c',"LineWidth",1.5)
grid on
legend("wc","w","w_{ref}","w_{est}")
ylabel('\psi (rad/s)'); xlabel('Time (Sec.)');

%%
figure(8)
subplot(1,2,1)
plot(t(1:length(A_E_P)),A_E_P',"-",'linewidth',1.5)
% legend("wc","w","w_{ref}")
xlabel('Time (Sec.)');
title ("Estimated values of A") , grid on

subplot(1,2,2)
plot(t(1:length(B_E_P)),B_E_P',"-",'linewidth',1.5)
% legend("wc","w","w_{ref}")
xlabel('Time (Sec.)');
title ("Estimated values of B") , grid on

%%
figure(9)
subplot(1,2,1)
plot(t(1:length(A_E)),A_E',"-",'linewidth',2)
% legend("wc","w","w_{ref}")
xlabel('Time (Sec.)');
title ("Disturbed values of A")

subplot(1,2,2)
plot(t(1:length(B_E)),B_E',"-",'linewidth',2)
% legend("wc","w","w_{ref}")
xlabel('Time (Sec.)');
title ("Disturbed values of B")

%%
%figure(10)

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(dt)),dt,'Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$t_{sample}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Sample Time}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

%%
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(Tu)),Tu,'linewidth',1); hold on
grid on;
legend({'$T_{u}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Disturbance}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

%%  
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(vref)),vref(:,:),'--','linewidth',1); hold on


grid on;
legend({'$T_{u}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Disturbance}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

