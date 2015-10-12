close all
clear all
home

%%% Initial conditions
tspan=[0 2.5]; dt=0.001;
x0 = [.5904+.077 0]';

%%% Identify your EOM function, policy function, and a force disturbance.
eom=@(t,x,u) Example_EOM_fun(x,u);
policy=@(t,x) Example_policy_teststandPart5(t,x);
%%% Default force disturbance is zeros.
disturbance=@(t,x) zeros(2,1);

%%%%% Discretize policy, run simulation. You shouldn't need to touch this
%%%%% section.
discrete_policy=@(t,x) discretize(policy,tspan,dt,t,x);
sys=@(t,x)vect4auto1(eom,discrete_policy,disturbance,t,x);
options=odeset('RelTol',1e-5,'MaxStep',dt);
tic; [T,X]=ode45(sys,tspan,x0,options); toc
[U_T,U]=discretize('Get Control History');
%%%%%


%%% Post-processing
plot_basic_teststand

%%% Clear persistent variables
% clear functions





