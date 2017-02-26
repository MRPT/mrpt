% Usage: viewPTG('./reactivenav.logs/PTGs',1) 
function [] = viewPTG(dir, nPTG)

% Load data
xs=load(sprintf('%s/PTG%i_x.txt',dir,nPTG));
ys=load(sprintf('%s/PTG%i_y.txt',dir,nPTG));
phis=load(sprintf('%s/PTG%i_phi.txt',dir,nPTG));
ds=load(sprintf('%s/PTG%i_d.txt',dir,nPTG));

% Dims:
nTrajs  = size(xs,1);
nPoints = size(xs,2);

% Check sizes:
assert(size(ys,1)==nTrajs);
assert(size(phis,1)==nTrajs);
assert(size(ds,1)==nTrajs);

% Draw paths:
DECIM = 1;  % Decimation (dont draw all)

figure(1);
clf;
for alpha=1:DECIM:nTrajs,
  x=xs(alpha,:);
  y=ys(alpha,:);
  plot(x,y,'-k'); hold on; 
end
axis equal;
title('Paths (x,y)');

figure(2);
clf;
for alpha=1:50:nTrajs,
  plot(ds(alpha,:)); hold on; 
end
title('distance over timesteps for each path');

end
