close all
clear all
clc

% Initial conditions
tspan=[0 5]; dt_inner=0.001; dt_outer=0.01;
x0=zeros(12,1); x0(7)=pi/6;%x0(8)=pi/6; x0(8)=pi/6;

% Identify your EOM function, policy functions and a force disturbance. 
eom=@(t,x,u) Quad_EOM(x,u);
inner_policy=@(t,x,u) Inner(t,x,u);
outer_policy=@(t,x) Outer(t,x,dt_outer);

% Default force disturbance is the identity (no disturbance)
disturbance=@(t,f)f;


%%% Discretize policy, run simulation. You shouldn't need to touch this
%%% section.
discrete_policy=@(t,x) discretize(inner_policy,outer_policy,tspan,dt_inner,dt_outer,t,x);
sys=@(t,x)vect12auto4(eom,discrete_policy,disturbance,t,x);
options=odeset('RelTol',1e-5,'MaxStep',min(dt_inner,dt_outer));
tic; [T,X]=ode45(sys,tspan,x0,options); toc
[Ui_T,Ui,Uo_T,Uo]=discretize('Get Control History');
%%%


% Plot results
plot_basic

% Clear persistent variables
clear functions