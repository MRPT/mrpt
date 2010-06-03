close all;
clear all;

vs = load('vs2.txt');
ws = load('ws2.txt');


subplot(221);
surf(vs);
colormap(gray);

subplot(222);
surf(ws);
colormap(gray);
