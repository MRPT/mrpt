close all;
clear all;

D=load('log_GT_vs_EKF.txt');

N=size(D,1);

GT_X=D(:,1); 
GT_Y=D(:,2);

EKF_X=D(:,3); 
EKF_Y=D(:,4); 

STD_X=D(:,5); 
STD_Y=D(:,6); 

subplot(2,1,1);
% X --------- 
% Area:
SIGMAS = 3;
ns= 1:N;
Xs1=(EKF_X+SIGMAS*STD_X)';
Xs2=(EKF_X-SIGMAS*STD_X)';

Ys1=(EKF_Y+SIGMAS*STD_Y)';
Ys2=(EKF_Y-SIGMAS*STD_Y)';

Xs=[Xs1 Xs2(end:-1:1) Xs1(1)];
ns_Xs=[ns ns(end:-1:1) ns(1)];

H=fill(ns_Xs,Xs,.9*[1 1 1]);
%set(H,'FaceColor',.9*[1 1 1]);
hold on;

% GT:
set(plot(ns,GT_X,'k'),'LineWidth',3);
set(plot(ns,EKF_X,'k--'),'LineWidth',2);
ylabel('X coordinate (meters)');
xlabel('Time steps');

subplot(2,1,2);
% Y --------- 
Ys=[Ys1 Ys2(end:-1:1) Ys1(1)];
ns_Xs=[ns ns(end:-1:1) ns(1)];

H=fill(ns_Xs,Ys,.9*[1 1 1]);
%set(H,'FaceColor',.9*[1 1 1]);
hold on;

% GT:
set(plot(ns,GT_Y,'k'),'LineWidth',3);
set(plot(ns,EKF_Y,'k--'),'LineWidth',2);
ylabel('Y coordinate (meters)');
xlabel('Time steps');
