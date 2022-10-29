clc
clear all
load('linssmodel.mat');
%X0=[0.5;0.25;-0.15;-0.25];    Use this to change initial state

t=input('Enter total time t :'); % This is Simulation Time
%======================Choose M/t Model===============
choice=1;%No Disturbance
%choice=2;%Random Noise               For question (f)
%choice=3;%With Constant bias

%=====================Kalman Filter Parameters========
MN = randi(4,4);
I=eye(4);
P0 =((0.2^2)*(MN*MN'))+ diag([2; 2; 2; 2]);
Q = 100*diag(X0);
R = 10000*eye(3);

%=======================MPC Parameters================
P=20; %Prediction Horizon
M=4; %Control Horizon

W_y=[1 0 0;0 1 0;0 0 1]; %Output Weight Matrix
W_u=[0 0;0 0]; %Input Weight Matrix
W_delu=[0 0;0 0]; %Delta U Weight Matrix

steps=round(t/Ts)
C=C_new;
[m,n]=size(C); %m=No.of Outputs
U_passst=zeros(2,steps);
youtputst=zeros(3,steps);

y0=[0.025;0.19;225];

%=================Reference Trajectory================
Y_sp=[0.03;0.25;325];
Y_ref=zeros(m,P);
Y_spplot=zeros(3,steps);
for i=1:P
    Y_ref(:,i)=1*(Y_sp); 
    %Y_ref(:,i)=i*(Y_sp)/P; %Use this instead for desired linear trajectory
end

%======================MPC============================
uprev0=zeros(M,2);
for s=1:1:steps 
    s

ypred1(uprev0,A,B,C,X0,P,M,W_y,W_u,W_delu,Y_ref); %Objective Fn


lb=0*ones(1,2*M)';
ub=2000*ones(1,2*M)';
opts=optimset('Algorithm','active-set','TolFun','0','TolCon','0','PlotFcns',[]);
U_opt1=fmincon(@(u)ypred1(u,A,B,C,X0,P,M,W_y,W_u,W_delu,Y_ref),uprev0,[],[],[],[],lb,ub,[],opts);
U_pass=U_opt1(1,:)';  
youtput; % question (b) and (f)
kalmanfilter;

uprev0=U_opt1;

U_passst(:,s)=U_pass;
youtputst(:,s)=y_opt';
Y_spplot(:,s)=(1*Y_sp);
end



%================Plotting==============================
Time=1:steps;
figure
plot(Time,U_passst(1,:),'-b',Time,U_passst(2,:),'--r');
xlabel('Steps')
ylabel('Control Moves')
legend('U1','U2')
figure
plot(Time,youtputst(1,:),'k',Time,Y_spplot(1,:),'b');
xlabel('Steps')
ylabel('Y(1)')
legend('Y1','Ysp1')
figure
plot(Time,youtputst(2,:),'--g',Time,Y_spplot(2,:),'b');
xlabel('Steps')
ylabel('Y(2)')
legend('Y2','Ysp2')
figure
plot(Time,youtputst(3,:),':m',Time,Y_spplot(3,:),'b');
xlabel('Steps')
ylabel('Y(3)')
legend('Y3','Ysp3')