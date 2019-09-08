close all;
clear;

global x y z

filname='kitti_raw.txt';
D=load(filname);
x=D(:,1);
y=D(:,2);
z=D(:,3);

pos0=[0 0 0.20];

N=14;
xs=linspace(-0.30,0.30,N);
ys=linspace(-0.30,0.30,N);
zs=linspace(0.10,0.30,N);

[X,Y,Z] = meshgrid(xs,ys,zs);

val=arrayfun(@(x1,x2,x3) xyz2elev_for_optim([x1,x2,x3]), X, Y,Z);

[v,loc] = min(val(:))
[ii,jj,kk] = ind2sub(size(val),loc)
sensor_pt=[xs(ii), ys(jj), zs(kk)]

%x=fminunc(@xyz2elev_for_optim, pos0, options)

