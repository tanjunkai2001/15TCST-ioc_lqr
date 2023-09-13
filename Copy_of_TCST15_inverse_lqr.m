% % Inverse LQR Control
%% Initialize params
clc;clear all; close all;

A = [100 0 -1
     0 0.1 50
     0.333 10 0];

B = [-1 0 10
    1 1 0
    0.1 -20 4];

n = size(A,1);%degrees of freedom
m = size(B,2);%control inputs

% Q = [71.4 96.4 62.8
%      96.4 302 8.62
%      62.8 8.62 1550];
% R = [1.33 -1.49 -3.19
%      -1.49 387 -77
%      -3.19 -77 53.3];
Q = 5*eye(3);
R = eye(3);
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
% Ke = [-3.75 27.6 45.3
%       3.93 -0.276 -1.7
%       19.7 2.98 4.57];% + randn(n,m);
Ke = K
Ke = K + .01*randn(n,m);
norm(K-Ke)

% clear Q R;

% using yalmip package, and sedumi solver
Q=sdpvar(n,n,'full'); R=sdpvar(m,m,'full') ;P=sdpvar(n,n,'full') ;
% P1=sdpvar(n,n,'full');
a=sdpvar(1,1,'full');

c1=[A'*P + P*A - P*B*Ke + Q == 0];
% c2=[(A-B*Ke)'*P + P*(A-B*Ke) + Ke'*R*Ke + Q == 0];
% c2=[P >= 0];
c3=[B'*P - R*Ke == 0];
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
P=value(P)
k=lqr(A,B,Q,R) % output of inverse Q R 
K               %given input 
norm(k-K)
matrix_type = check_matrix_definiteness(P)


