function [] = xyzi_to_range_depth(filname)

if nargin<1
    filname='kitti_raw.txt';
end

SHOW_3D_POINTS=0;
SHOW_RANGE_IMG=1;

% Value obtained by optimization:
sensorPt = [ -0.0062    0.0273    0.1757];
%sensorPt = [ 0 0 0];

D=load(filname);
x=D(:,1);
y=D(:,2);
z=D(:,3);
I=D(:,4);

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

if (SHOW_RANGE_IMG)
    figure(1);
    scatter(azimuth*180/pi,elevation*180/pi,5,I);
    grid on;
    xlabel('Azimuth (deg)');
    ylabel('Elevation (deg)');
    colormap gray;
    colorbar;
end


if (SHOW_3D_POINTS)
    figure(2);
    plot3(x,y,z,'.','MarkerSize',5);
    axis equal;
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
end

end