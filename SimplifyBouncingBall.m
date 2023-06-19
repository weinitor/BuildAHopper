% SIMPLIFY_BOUNCING_BALL - Model the movement of a bouncing ball using the
% coefficient of restitution to discount its potential energy.

% Written by Wei-Hsi Chen <weicc@seas.upenn.edu>
% Last Edited 06/17/2023
%
% Copyright (C) 2023 The Trustees of the University of Pennsylvania. 
% All rights reserved. Please refer to LICENSE.md for detail.

close all; clear all; clc;

% Add all the folders and subfolders to the search path
addpath(genpath(fileparts(mfilename('fullpath'))));

% Asigning physical specification
param.g = 9.8;
param.r = 0.05;         % ball diameter/ spring rest length
param.m = 1;            % ball mass
param.discount = 0.8;   % Energey discount

% Initial position Q0 = [q dq]
Q0 = [1 0];
tbegin = 0;
tfinal = 5;

% ODE solving
T = tbegin;
Q = Q0;
while T(end) < tfinal
    options = odeset('Events',@(t, Q)EventTouchDown(t, Q, param));
    [Ttemp, Qtemp, te, Qe, ie] = ode45(@(t, Q)EOMFlight(t, Q, param),...
            [tbegin, tfinal], Q0, options);
    nT= length(Ttemp);
    T = [T; Ttemp(2:nT)];
    Q = [Q; Qtemp(2:nT,:)];
    Q0(1) = Qtemp(nT,1);
    Q0(2) = -sqrt(param.discount)*Qtemp(nT,2);
    tbegin = T(end);
end
    
% Interpolation of Times
T2 = [0: 1e-2: tfinal];
Q2 = interp1(T, Q, T2);
% Simulation Visualization
RunBallSimulation(T2, Q2, param, 'P21SimplifyBouncingBall');