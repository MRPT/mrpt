close all;
clear;

global x y z

filname='kitti_raw.txt';
D=load(filname);
x=D(:,1);
y=D(:,2);
z=D(:,3);

options=optimset(...
    'Display','iter',...
    'TolFun', 1e-10,...
    'TolX', 1e-9...
    );

pos0=[0 0 0.10];
x=fminunc(@xyz2elev_for_optim, pos0, options)

