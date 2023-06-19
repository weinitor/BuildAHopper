% ACTIVE_HOPPER - Model the movement of a vertical hopper as a
% spring-mass-damper system. This model includes a constant thrust input at
% the bottom of the stance phase to compenstate for the energy lost to the
% damper.

% Written by Wei-Hsi Chen <weicc@seas.upenn.edu>
% Last Edited 06/17/2023
%
% Copyright (C) 2023 The Trustees of the University of Pennsylvania. 
% All rights reserved. Please refer to LICENSE.md for detail.

close all; clear all; clc;

% Add all the folders and subfolders to the search path
addpath(genpath(fileparts(mfilename('fullpath'))));

% Asigning robot specification
param.g = 9.8;
param.d = 0.08;     % ball diameter
param.r = 0.3;      % spring rest length/ chi0
param.m = 3;        % ball mass
param.k = 5000;     % spring eleastic constant
param.b = 10;        % damping constant
param.tau = 0;    % thrust
param.thrusttime = 0.005;  % duration of thrust

% Initial position Q0 = [q dq]
Q0 = [1 0];
tbegin = 0;
tfinal = 5;
phase = 4;

% ODE solving
T = tbegin;
Q = Q0;
while T(end) < tfinal
    if phase == 1
        % touch down, into stance phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventBottom(t, Q, param)));
        phase = 2;
    elseif phase == 2
        % stance phase, injecting thrust
        [Ttemp, Qtemp] = ...
            ode45(@(t, Q)EOMStanceThrusted(t, Q, param),...
            [tbegin, tbegin + param.thrusttime], Q0);
        phase = 3;
    elseif phase == 3
        % stance phase without thrust
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventLiftOff(t, Q, param)));
        phase = 4;
    elseif phase == 4
        % lift off, into fly phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMFlight(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventTouchDown(t, Q, param)));
        phase = 1;
    end
    nT= length(Ttemp);
    T = [T; Ttemp(2:nT)];
    Q = [Q; Qtemp(2:nT,:)];
    Q0 = Qtemp(nT,:);
    tbegin = Ttemp(nT);
end
    
T2 = [0: 1e-2: tfinal];
Q2 = interp1(T, Q, T2);
% Simulation Visualization
RunHopperSimulation(T2, Q2, param, 'none');

end