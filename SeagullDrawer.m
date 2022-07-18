function [Seagull,x,y] = SeagullDrawer()

hold on
wings = [0.550000 0.570000 0.670000];
color = [0.950000 0.950000 0.960000];
beak = [0.930000 0.570000 0.130000];

t=0:0.01:2*pi;

x1=3+2 * cos(t);
y1=0.1+0.6*sin(t);
Seagull = [fill(x1,y1,color)];
x{1} = x1;
y{1} = y1;

x2=3.4+0.4*cos(t);
y2=1.2-1+0.2*sin(t);
Seagull = [Seagull, fill(x2,y2,'w')];
x{2} = x2;
y{2} = y2;

x3=2.6+0.4*cos(t);
y3=1.2-1+0.2*sin(t);
Seagull = [Seagull, fill(x3,y3,'w')];
x{3} = x3;
y{3} = y3;

x4=3.45+0.2*cos(t);
y4=1.1-1+0.1*sin(t);
Seagull = [Seagull, fill(x4,y4,'k')];
x{4} = x4;
y{4} = y4;

x5=2.65+0.2*cos(t);
y5=1.1-1+0.1*sin(t);
Seagull = [Seagull, fill(x5,y5,'k')];
x{5} = x5;
y{5} = y5;

x6 = flip([4.3 3 3.0000]);
y6 = [-0.0990+0.12 -0.0990-0.05 -0.0343];
Seagull = [Seagull, fill(x6,y6,beak)];
x{6} = x6;
y{6} = y6;

x7 = [1.04 1.55 0.05];
y7 = [ -0.034 0.44 0.4];
Seagull = [Seagull, fill(x7,y7,color)];
x{7} = x7;
y{7} = y7;

x8 = [0.05 1.04 -2];
y8 = [0.4 -0.034 0];
Seagull = [Seagull, fill(x8,y8,wings)];
x{8} = x8;
y{8} = y8;

x9 = [0.05 -2 -4];
y9 = [0.4 0 0.5];
Seagull = [Seagull, fill(x9,y9,wings)];
x{9} = x9;
y{9} = y9;

x10 = [4.5 4.93 6.05];
y10 = [0.43 -0.034 0.4];
Seagull = [Seagull, fill(x10,y10,color)];
x{10} = x10;
y{10} = y10;

x11 = [4.93 6.05 8];
y11 = [-0.034 0.4 0];
Seagull = [Seagull, fill(x11,y11,wings)];
x{11} = x11;
y{11} = y11;

x12 = [6.05 8 10];
y12 = [0.4 0 0.5];
Seagull = [Seagull, fill(x12,y12,wings)];
x{12} = x12;
y{12} = y12;

x13 = [8 10 13];
y13 = [0 0.5 0.35];
Seagull = [Seagull, fill(x13,y13,wings)];
x{13} = x13;
y{13} = y13;

x14 = [-2 -4 -7];
y14 = [0 0.5 0.35];
Seagull = [Seagull, fill(x14,y14,wings)];
x{14} = x14;
y{14} = y14;

end