% % Inverse LQR Control
%% Initialize params
clc;clear all; close all;

A = [0 1 7 9
     4 -8 -5 -3
     8 -7 7 -6
     10 -5 -5 -5];

B = [2 2 5 -9
     -1 1 5 -9
     -3 9 -3 1
     7 -4 1 6];

n = size(A,1);%degrees of freedom
m = size(B,2);%control inputs

Q = [13.9 -1.32 3.9 2.65
     -1.32 7.35 3.72 -2.27
     3.9 3.72 5.28 0.373
     2.65 -2.27 0.373 4.33];
R = [10.6 0.302 6.56 -1.83 
     0.302 4.69 2.06 3.87
     6.56 2.06 7.17 -0.266
     -1.83 3.87 -0.266 6.88];
Q = 5*eye(n);
R = eye(m);
% xo = [0.5 0.5 0.5 0.5];

C=eye(n);

% check controlability
if(rank(ctrb(A,B))==n)
    disp('system is controllable');
else
    error('system is not controllable');
end



%% Solve IOC
K=lqr(A,B,Q,R); % let say we know this K before then we need to get that same Q and R
Ke = [2.362 -1.32 1.186 1.836
     4.293 -0.8472 3.789 1.538
     -2.252 1.339 -2.081 -1.312
     -2.707 -0.06996 -2.062 -0.72633];% + randn(n,m);
Ke = K + 0.01*randn(n,m)
norm(K-Ke)

% clear Q R;

% using yalmip package, and sedumi solver
Q=sdpvar(n,n,'full'); R=sdpvar(m,m,'full') ;P=sdpvar(n,n,'full') ;
% P1=sdpvar(n,n,'full');
a=sdpvar(1,1,'full');

e = 0.02
c1=[A'*P + P*A - P*B*Ke + Q == 0];
c1=[-e <= A'*P + P*A - P*B*Ke + Q <= e];

% c2=[(A-B*Ke)'*P + P*(A-B*Ke) + Ke'*R*Ke + Q == 0];
% c2=[P >= 0];
c3=[B'*P - R*Ke == 0];
c3=[-e <= B'*P - R*Ke <= e];

% c4=[A'*P1 + P1*A' <= Q]; % 这里需要避免使用"strict inequality"，参考YALMIP的一篇blog：https://yalmip.github.io/inside/strictinequalities/

c4=[[[Q zeros(n,m)];[zeros(m,n) R]] >= eye(m+n)];
c5=[[[Q zeros(n,m)];[zeros(m,n) R]] <= a.*eye(m+n)];

% constraints=[c1,c2,c3,c4,c5];
% constraints=[c1,c2,c3,c5];
constraints=[c1,c3,c4,c5];

optimize(constraints, a^2)  % 优化a^2，同时满足约束constraints



%% display value
Q=value(Q)
R=value(R)
% P=value(P)
k=lqr(A,B,Q,R) % output of inverse Q R 
% K               %given input 


% norm(k-K)



