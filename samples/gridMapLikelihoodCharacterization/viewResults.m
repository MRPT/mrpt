figure;

bmp = imread('test_gridmap2.bmp');

subplot(221);
imagesc(bmp);
title('El mapa');

subplot(222);
l=load('out_lik.txt');
imagesc(l), colormap(gray);
title('Valor del likelihood field en 2D');


subplot(2,2,3);
h=surf((l./max(l(:)))*50);
set(h,'LineStyle','none');
title('Valor del likelihood field en 3D');
axis equal;

subplot(224);
plot(l(100,:))
title('Corte transversal:');
