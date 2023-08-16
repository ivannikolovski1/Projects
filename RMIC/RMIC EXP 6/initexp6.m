%% Init

close all;clear;clc;

g   = 9.81;

J  = 0.001;
d  = 0.1;
ke = 0.1;
km = 0.1;
L  = 0.01;
R  = 0.1;
m  = 1;
l  = 0.1375;



%% TASK 1
A=[0 1 0;
    -m*g*l/(J+m*l^2) -d/(J+m*l^2) km/(J+m*l^2);
    0 -ke/L -R/L];
B=[0;0;1/L];
C=[1 0 0];
D=0;
sys=ss(A,B,C,D);
q=10*pole(sys);
h=place(A',C',q).';
%% TASK 2
Qs=ctrb(sys);
rank(Qs); %controlable because full rank
syms s;
pol=det(eye(size(A))*s-A);
a=sym2poly(pol);
a=flip(a);
W=[a(2) a(3) 1;
    a(3) 1 0;
    1 0 0];
T=Qs*W;
WantedPol=(s+10)*(s+10)*(s-20)
astar=flip(sym2poly(WantedPol));%flips coeffeicents so a0 is first and an last
krT=a-astar;
krT=krT(1:3);
kT=krT*inv(T);