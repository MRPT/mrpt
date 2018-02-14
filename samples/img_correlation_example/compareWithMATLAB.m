clear all;
close all;

i1=load('i1.txt');
i2=load('i2.txt');

I1_r=load('I1_R.txt'); I1_i=load('I1_I.txt');
I2_r=load('I2_R.txt'); I2_i=load('I2_I.txt');

I1_r=load('I1_R.txt');I1_i=load('I1_I.txt'); 
I1=I1_r+i*I1_i;

I2_r=load('I2_R.txt');I2_i=load('I2_I.txt'); 
I2=I2_r+i*I2_i;

DIV_MATLAB = I2./I1;
DIV_MATLAB_R = real(DIV_MATLAB);
DIV_MATLAB_I = imag(DIV_MATLAB);

DIV_C_R = load('DIV_R.txt');
DIV_C_I = load('DIV_I.txt');

%imagesc(abs(CORR)),colormap(gray);