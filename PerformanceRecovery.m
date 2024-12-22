clc; clear all;

ft = 20;        %Total Sim Time
dt = 0.005;     %First-order Euler method's discritization sample 

x = 0;          %Initial condition of x
xr = 0;         %Initial condition of xr
xri = 0;        %Initial condition of xri
psi = 0;        %Initial condition of psi 
w_hat = zeros(6,1);     %Initial condition of w_hat

alp = 2;        %Nominal control law
gam = 50;       %Adaptive control law
sig = 0.2;      %Leakage term 
lam = 1;      %Performance recovery 

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

    e = x - xr;                         %Error 
    un = -alp*(x-c);
    ua = -w_hat'*theta;
    psi = psi + dt*(-lam*(psi-e));
    v = (lam*psi-(alp+lam)*e);     %Corrective signal 
    u = un+ua+v;
    xr = xr + dt*(-alp*(xr-c) + v);     %Reference Model 
    xri = xri + dt*(-alp*(xri-c));      %Ideal closed-loop performance 

    w_hat = w_hat + dt*(gam*(theta*e -sig*w_hat));

    x = x+ dt*(delta + u);

    %Data Recording 
    x_rec(idx,1:1) = x;     xri_rec(idx,1:1) = xri;
    xr_rec(idx,1:1) = xr;   u_rec(idx,1:1) = u;     
    d_rec(idx,1:1) = delta; dhat_rec(idx,1:1) = ua; 
    t_rec(idx,1:1) = t;     idx = idx + 1;
end;

%Plot
figure
subplot(2,1,1);hold on; box on;
p0 = plot(t_rec,xri_rec,'g'); set(p0,'LineWidth',3);
%p1 = plot(t_rec,xr_rec,'k'); set(p1,'LineWidth',3);
p2 = plot(t_rec,x_rec,'k'); set(p2,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$x(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;

subplot(2,1,2);hold on; box on;
p2 = plot(t_rec,u_rec,'k'); set(p2,'LineWidth',3);
xlabel({'$t$ (sec)'},'FontSize',16,'Interpreter','latex');
ylabel({'$u(t)$'},'FontSize',16,'Interpreter','latex'); axis tight;

