M = 1;    % mass of the chassis  
m = 0.2;  % mass of the wheels and shaft
b = 0.1;  % estimate of viscous friction coefficient (N-m-s)
I = 0.0005;% moment of inertia of the pendulum
g = 9.8;  %acceleration due to gravity (m/s^2)
l = 0.125;  %length to pendulum center of mass
%where is radius????
speed1=10;
p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];

B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];

C = [1 0 0 0;
     0 0 1 0];

D = [0;
     0];



Q = C'*C;
Q(1,1) = 5000;
Q(3,3) = 100
R = 1;

poles_before_feedback=eig(A) 
K = lqr(A,B,Q,R)





Ac = [(A-B*K)]; 
poles_after_feedback=eig(Ac)

Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};



sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:20;
r =-0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')





% 
% 
% 
% ob = obsv(sys_ss);
% observability = rank(ob)
% poles = eig(Ac)
% P = [-40 -41 -42 -43];
% L = place(A',C',P)'
% 
% 
% 
% Ace = [(A-B*K) (B*K);
%        zeros(size(A)) (A-L*C)];
% Bce = [B*Nbar;
%        zeros(size(B))];
% Cce = [Cc zeros(size(Cc))];
% Dce = [0;0];
% 
% states = {'x' 'x_dot' 'phi' 'phi_dot' 'e1' 'e2' 'e3' 'e4'};
% inputs = {'r'};
% outputs = {'x'; 'phi'};
% 
% sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% t = 0:0.01:5;
% r = 0.2*ones(size(t));
% [y,t,x]=lsim(sys_est_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','cart position (m)')
% set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% title('Step Response with Observer-Based State-Feedback Control')
