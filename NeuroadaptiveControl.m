%Following Proffesor Tansel Yucelen Youtube Lectures 
clc; clear all;

ft = 20;        %Total Sim Time
dt = 0.005;     %First-order Euler method's discritization sample 

x = 0;          %Initial condition of x
xr = 0;         %Initial condition of xr
w_hat = zeros(6,1);     %Initial condition of w_hat

alp = 2;        %Nominal control law
gam = 50;       %Adaptive control law
sig = 0.2;      %Leakage term 

%Sim
idx = 1;
for t = 0:dt:ft;
    delta = 1 + x^2 + sin(x)*(x^3) + cos(x)*(x^4); %Uncertainty 

    if t <= 5;           c = 1; end;
    if t > 5 && t <= 10; c = -1; end;
    if t >10 && t <= 15; c = 1; end;
    if t > 15;           c = -1; end;

    %6 neurons 
    theta = [exp(-0.25*(abs(x-5))^2);
             exp(-0.25*(abs(x-3))^2);
             exp(-0.25*(abs(x-1))^2);
             exp(-0.25*(abs(x+1))^2);
             exp(-0.25*(abs(x+3))^2);
             exp(-0.25*(abs(x+5))^2)];

    un = -alp*(x-c);
    ua = -w_hat'*theta;
    u = un+ua;
    xr = xr + dt*(-alp*(xr-c));

    w_hat = w_hat + dt*(gam*(theta*(x-xr)-sig*w_hat));

    x = x+ dt*(delta + u);

    %Data Recording 
    x_rec(idx,1:1) = x;     xr_rec(idx,1:1) = xr;
    u_rec(idx,1:1) = u;     d_rec(idx,1:1) = delta;
    dhat_rec(idx,1:1) = ua; t_rec(idx,1:1) = t;
    idx = idx + 1;
end;

%Plot
figure
subplot(3,1,1);hold on; box on;
p0 = plot(t_rec,xr_rec,'g'); set(p0,'LineWidth',3);
p1 = plot(t_rec,x_rec,'k'); set(p1,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$x(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;

subplot(3,1,2);hold on; box on;
p2 = plot(t_rec,u_rec,'k'); set(p2,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$u(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;

subplot(3,1,3);hold on; box on;
p3 = plot(t_rec,d_rec+dhat_rec,'k'); set(p3,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$\tilde{\delta}(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;
