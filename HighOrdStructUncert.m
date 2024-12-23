clc; clear all; close all;

ft = 40;                %Total sim time
dt = 0.005;             %1st-order Euler method's discretization sampling time

mass = 5;               %Mass
alpha = 2.5;         %Spring
beta = 1;            %Damper
A = [0 1; 0 0];
B = [0;1];
Lam = [1/mass];     %Lambda
W0 = [-alpha;-beta];
x = zeros(2,1);      %Initial condition of x

Q = eye(2);
R = 0.1;
K1 = lqr(A,B,Q,R);
K2 = -1/([1 0]*inv(A-B*K1)*B);
Ar = A-B*K1;
Br = B*K2;
xr = zeros(2,1);     %Initial condition of xr 

gam = 0;             %Adaptation gain
P = lyap(Ar',Q);     %Solving ARE
W_hat = zeros(3,1);  %Initial condition of w_hat

%Sim
idx = 1;
for t = 0:dt:ft;

    if t <= 10;           c = 1;  end;
    if t > 10 && t <= 20; c = -1; end;
    if t > 20 && t <= 30; c = 1;  end;
    if t > 30;            c = -1; end;

    un = -K1*x + K2*c;      %Nominal control
    ua = -W_hat'*[x(1);x(2)^3;un];  %Adaptation
    u = un + ua;
    e = x - xr;
    xr = xr + dt*(Ar*xr + Br*c);
    W_hat = W_hat + dt*(gam*[x(1);x(2)^3;un]*e'*P*B);

    x = x + dt*(A*x + B*Lam*(u+W0'*[x(1);x(2)^3]));

    %Data Recording 
    x_rec(idx,1:2) = x;     xr_rec(idx,1:2) = xr;
    u_rec(idx,1:1) = u;     c_rec(idx,1:1) = c;
    t_rec(idx,1:1) = t;     idx = idx + 1;
end;

%Plot
figure
subplot(3,1,1);hold on; box on;
p0 = plot(t_rec,c_rec,'g'); set(p0,'LineWidth',3);
p1 = plot(t_rec,xr_rec(:,1),'k'); set(p1,'LineWidth',3);
p2 = plot(t_rec,x_rec(:,1),'k'); set(p2,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$x_1(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;

subplot(3,1,2);hold on; box on;
p0 = plot(t_rec,xr_rec(:,2),'c'); set(p0,'LineWidth',3);
p2 = plot(t_rec,x_rec(:,2),'k'); set(p2,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$x_2(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;

subplot(3,1,3);hold on; box on;
p3 = plot(t_rec,u_rec,'k'); set(p3,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$u(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;
