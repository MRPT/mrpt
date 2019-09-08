function [err] = xyz2elev_for_optim(sensorPt)

global x y z
N=length(x);

lx=x-sensorPt(1);
ly=y-sensorPt(2);
lz=z-sensorPt(3);
R=zeros(N,1);

azimuth=zeros(N,1);
elevation=zeros(N,1);
for i=1:N,
    R(i) = sqrt(lx(i)^2 + ly(i)^2+ lz(i)^2);
    azimuth(i) = atan2(ly(i),lx(i));
    elevation(i) = atan2(lz(i),hypot(lx(i),ly(i)));
end

% evaluate elevation std dev. in some segments
% (if sensor position is 0, it should be zero):
tst_indices=[...
    2102, 4064;...
    43140, 45180;...
    11390, 11550;...
    66880, 68720;...
    71120, 73030;...
    90490, 92030;...
    69080, 70180;...
    112100, 113700;...
    69030, 70870;...
    75560, 77320;...
    ];
errs=[];
for i=1:size(tst_indices,1)
    elevs = elevation(tst_indices(i,1):tst_indices(i,2));
    errs=[errs; std(elevs)];
end
err = sqrt(mean(errs.^2));

end