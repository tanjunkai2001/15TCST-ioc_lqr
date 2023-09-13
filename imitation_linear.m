clc; clear all; close all;
%define linear system parameters
A = [2, -2; 1, 0];
B = [1; 1];
n = size(A,1);%degrees of freedom
m = size(B,2);%control inputs
C=eye(2);
Q = 5*eye(n);
R = 2*eye(m);
k=lqr(A,B,Q,R);
%check controlability
if(rank(ctrb(A,B))==n)
    disp('system is controllable');
else
    error('system is not controllable');
end
xo = [50; 0];% initial condition
% k=[0,0];

x1=[];x2=[]; %trajectories list
ti=[]; %times list
%generate different trajectories for the simulation 
for i=0.1:0.1:1
    [t,x]=ode45(@(t,x)linear_ode(A,B,C,k+i,t,x),[0:0.01:20],xo);
    ti=t';
    x1=[x1;x(:,1)'];
    x2=[x2;x(:,2)'];
end
x1=mean(x1);
x2=mean(x2);
ord=10;
%fit polynomial curve to the data
figure(1)
p1=polyfit(ti,x1,ord);
plot(ti,polyval(p1,ti));
figure(2);
p2=polyfit(ti,x2,ord);
plot(ti,polyval(p2,ti));


