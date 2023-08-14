clear all, clc,

import casadi.*

x1 = MX.sym('x1'); % States
x2 = MX.sym('x2');
x  = [x1; x2];    
u  = MX.sym('u'); % Controls

% Van der Pol oscillator system  (nonlinear ODE)
ode = [x2; 
       (34515869019144192*sin(x1) - 2333022232964875*u + 1337006139375616*sin(x1)*x2 - 1337006139375616*cos(x1)*u)/(70368744177664*(19*cos(x1) + 50))];

f = Function('f',{x,u},{ode},{'x','u'},{'ode'});
f([0.2;0.8],0.1);

T = 10; % Time horizon
N = 20; % Number of control intervals

% Integrator to discretize the system
intg_options = struct;
intg_options.tf = T/N;
intg_options.simplify = true;
intg_options.number_of_finite_elements = 4;

% DAE problem structure
dae = struct;
dae.x = x;         % What are states?
dae.p = u;         % What are parameters (=fixed during the integration horizon)?
dae.ode = f(x,u);  % Expression for the right-hand side

intg = integrator('intg','rk',dae,intg_options);

res = intg('x0',[0;1],'p',0);  % Easier to identify inputs, but bloated API
res.xf;

res = intg('x0',x,'p',u); % Evaluate with symbols
x_next = res.xf;

% Simplify API to (x,u)->(x_next)
F = Function('F',{x,u},{x_next},{'x','u'},{'x_next'});
F([0;1],0);
F([0.1;0.9],0.1);

F;
sim = F.mapaccum(N);

x0 = [0;1];
res = sim(x0,zeros(1,N));

figure
tgrid = linspace(0,T,N+1);
plot(tgrid,full([x0 res]));
legend('x1','x2');
xlabel('t [s]');

%% CONTROL

opti = casadi.Opti();

x = opti.variable(2,N+1); % Decision variables for state trajetcory
u = opti.variable(1,N);
p = opti.parameter(2,1);  % Parameter (not optimized over)

opti.minimize(sumsqr(x)+sumsqr(u));

for k=1:N
  opti.subject_to(x(:,k+1)==F(x(:,k),u(:,k)));
end
opti.subject_to(-50<=u<=50);
opti.subject_to(x(:,1)==p);

% Choose a concerete solver
opti.solver('sqpmethod',struct('qpsol','qrqp'));

% And choose a concrete value for p
opti.set_value(p,[-60;0].*pi/180); % Valores iniciales);
sol = opti.solve();

figure
hold on
plot(tgrid,sol.value(x));
stairs(tgrid, [sol.value(u) nan], '-.');
xlabel('t [s]');
legend('x1','x2','u');