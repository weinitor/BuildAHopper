% ACTIVE_ELECTROMAGNETIC_HOPPER_0816 - Model the movement of a vertical hopper
% as a system of mass, virtual spring and virtual damper. This model
% changes the stiffness of the virtual spring at the bottom of the stance
% phase to compenstate for the energy lost to the damper. This virtual
% spring system is often done by using a electromagnetic motor.
% This program only plots partial solution.

% Written by Wei-Hsi Chen <weicc@seas.upenn.edu>
% Last Edited 06/17/2023
%
% Copyright (C) 2023 The Trustees of the University of Pennsylvania. 
% All rights reserved. Please refer to LICENSE.md for detail.

close all; clear all; clc;

% Add all the folders and subfolders to the search path
addpath(genpath(fileparts(mfilename('fullpath'))));

close all; clear all; clc;

% Asigning robot specification
param.g = 9.8;
param.d = 0.08;     % ball diameter
param.r = 0.3;      % spring rest length/ chi0
param.m = 3;        % ball mass
param.k = 5000;     % vitual spring eleastic constant
param.b = 6;        % damping constant
param.N = 1.14;     % ratio of vitual spring eleastic constant at thrust
param.thrusttime = 0.005;  % duration of thrust

% Initial position Q0 = [q dq]
Q0 = [1 0];
tbegin = 0;
tfinal = 1;
phase = 4;

% ODE solving
T = tbegin;
Q = Q0;
E = 0;
thend = [];
eve=[];
while T(end) < tfinal
    if phase == 1
        % touch down, into stance phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventBottom(t, Q, param),'MaxStep',.001,'AbsTol',1e-10));
        Etemp=1/2*param.m*Qtemp(:,2).^2+param.m*param.g*Qtemp(:,1)+1/2*param.k*(Qtemp(:,1)-param.r).^2;
        phase = 2;
        eve=[eve;[te,Qe]];
    elseif phase == 2
        % stance phase, injecting thrust
        [Ttemp, Qtemp] = ...
            ode45(@(t, Q)EOMSpringThrust(t, Q, param),...
            [tbegin, tbegin + param.thrusttime], Q0);
        Etemp=1/2*param.m*Qtemp(:,2).^2+param.m*param.g*Qtemp(:,1)+1/2*param.N*param.k*(Qtemp(:,1)-param.r).^2;
        phase = 3;
        thend=[thend;[Ttemp(end),Qtemp(end,:)]];
    elseif phase == 3
        % stance phase without thrust
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventLiftOff(t, Q, param),'MaxStep',.001,'AbsTol',1e-10));
        Etemp=1/2*param.m*Qtemp(:,2).^2+param.m*param.g*Qtemp(:,1)+1/2*param.k*(Qtemp(:,1)-param.r).^2;
        phase = 4;
        eve=[eve;[te,Qe]];
    elseif phase == 4
        % lift off, into fly phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMFlight(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventTouchDown(t, Q, param),'MaxStep',.001,'AbsTol',1e-10));
        Etemp=1/2*param.m*Qtemp(:,2).^2+param.m*param.g*Qtemp(:,1);
        phase = 1;
        eve=[eve;[te,Qe]];
    end
    nT= length(Ttemp);
    T = [T; Ttemp(2:end)];
    Q = [Q; Qtemp(2:end,:)];
    E = [E; Etemp(2:end)];
    Q0 = Qtemp(end,:);
    tbegin = Ttemp(end);
end
T2 = [0: 1: tfinal];
Q2 = interp1(T, Q, T2);
% Simulation Visualization
% RunHopperSimulation(T2, Q2, param, 'none');
% figure(1)
% size(E)
% size(T)
% plot(T,E)
plot(T,Q(:,1),thend(:,1),thend(:,2),'r.',eve(:,1),eve(:,2),'k.')
figure(2)
plot(T,Q(:,2))