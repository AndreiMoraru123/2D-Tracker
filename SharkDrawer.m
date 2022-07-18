function [Shark,x,y] = SharkDrawer()

hold on
color = [0.370000 0.620000 0.630000];
t=0:0.01:2*pi;
x1=4*cos(t);
y1=0.5*sin(t);
x1 = 2 * x1;
y1 = 2 * y1;
Shark = fill(x1,y1,color);
x{1} = x1;
y{1} = y1;

x2=2.6+0.2*cos(t);
y2=1.2-1+0.2*sin(t);
x2 = 2 * x2;
y2 = 2 * y2;
Shark = [Shark, fill(x2,y2,'w')];
x{2} = x2;
y{2} = y2;

x3=3.4+0.2*cos(t);
y3=1.2-1+0.2*sin(t);
x3 = 2 * x3;
y3 = 2 * y3;
Shark = [Shark, fill(x3,y3,'w')];
x{3} = x3;
y{3} = y3;

x4=2.65+0.1*cos(t);
y4=1.2-1+0.1*sin(t);
x4 = 2 * x4;
y4 = 2 * y4;
Shark = [Shark, fill(x4,y4,'k')];
x{4} = x4;
y{4} = y4;

x5=3.45+0.1*cos(t);
y5=1.2-1+0.1*sin(t);
x5 = 2 * x5;
y5 = 2 * y5;
Shark = [Shark, fill(x5,y5,'k')];
x{5} = x5;
y{5} = y5;

x6 = [ 0 0.2 2];
y6 = [ 0.5 0.7 0.375];
x6 = 2 * x6;
y6 = 2 * y6;
Shark = [Shark, fill(x6,y6,color)];
x{6} = x6;
y{6} = y6;

x7=2.55 + cos(t);
y7= - 0.2 + 0.1*sin(t);
x7 = 2 * x7;
y7 = 2 * y7;
Shark = [Shark, fill(x7,y7,[0.960000 0.760000 0.760000])];
x{7} = x7;
y{7} = y7;

x8 = [ 1.73 2.15 2.6];
y8 = [ -0.14 -0.29 -0.10];
x8 = 2 * x8;
y8 = 2 * y8;
Shark = [Shark, fill(x8,y8,'w')];
x{8} = x8;
y{8} = y8;

x9 = [ 2.6 2.81 3.32];
y9 = [ -0.10 -0.29 -0.13];
x9 = 2 * x9;
y9 = 2 * y9;
Shark = [Shark, fill(x9,y9,'w')];
x{9} = x9;
y{9} = y9;

x10 = [ -0.55 0.2 0.41] + 5;
y10 = [ -0.6 -0.35 -0.6];
Shark = [Shark, fill(x10,y10,'w')];
x{10} = x10;
y{10} = y10;

x11 = [ 0.89 1.75 1.67] + 5;
y11 = [ -0.58 -0.3 -0.52];
Shark = [Shark, fill(x11,y11,'w')];
x{11} = x11;
y{11} = y11;

x12 = [ -0.55 0.2 0.43];
y12 = [ -0.6 -0.35 -0.2] + 0.3;
Shark = [Shark, fill(x12,y12,[0.960000 0.760000 0.760000])];
x{12} = x12;
y{12} = y12;

x13 = [ -0.55 0.2 0.43] + 0.5;
y13 = [ -0.6 -0.35 -0.2] + 0.3;
Shark = [Shark, fill(x13,y13,[0.960000 0.760000 0.760000])];
x{13} = x13;
y{13} = y13;

x14 = [ -0.55 0.2 0.43] - 0.5;
y14 = [ -0.6 -0.35 -0.2] + 0.3;
Shark = [Shark, fill(x14,y14,[0.960000 0.760000 0.760000])];
x{14} = x14;
y{14} = y14;
end