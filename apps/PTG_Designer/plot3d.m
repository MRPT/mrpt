function [] = plot3d(fich)
l=load(fich);
figure;
x=l(:,1);y=l(:,2);p=l(:,3);

plot3(0,0,0,'k.');
hold on;
K = 1;
for i=(2*K):K:length(x),
    c = [1 0.3 0];
%     if (p(i)>pi),
%         p(i)=2*pi-p(i);
%     end
%     if (p(i)<-pi),
%         p(i)=2*pi+p(i);
%     end
    cl = (p(i) + 3.5)/(7);
    cl = max(0,min(1,cl));
    c = c*cl;
    
    GR = 3;
%      if (i>=4311 & i<=4350),
%          GR = 5;
%          c=[0 0 0];
% %         p(i)=p(i)+0.1;
%      end    
    
    if (abs(x(i)-x(i-K))<1 & abs(y(i)-y(i-K))<1 & abs(p(i)-p(i-K))<1),
        set(line([x(i-K) x(i)],[y(i-K) y(i)],[p(i-K) p(i)]),'Color',c,'LineWidth',GR);
    end
end
grid on;

set(line([-0.1 0.1],[0 0],[0 0]),'Color',[0 0 0],'LineWidth',6);
set(line([0 0],[-0.1 0.1],[0 0]),'Color',[0 0 0],'LineWidth',6);
set(line([0 0],[0 0],[-0.1 0.1]),'Color',[0 0 0],'LineWidth',6);


% Velocidades:
figure;
Vs=load(sprintf('V_%s',fich));
a=Vs(:,1);
v=Vs(:,2);
w=Vs(:,3)*180/pi;
subplot(211), plot(a,v,'k'); 
if (min(v)==max(v)),
    minv=max(v)*1.1;
    maxv=0;
else
    minv=min(v);
    maxv=max(v)*1.1;
end
axis([-3.14 3.14 minv maxv]);

subplot(212), plot(a,w,'k'); 
axis([-3.14 3.14 min(w) max(w)]);


