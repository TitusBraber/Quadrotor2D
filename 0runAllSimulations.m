%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad
close all;
clear all;clc;

hbar=waitbar(0,'plotting');

%%
run('zFF_profile.m')
waitbar(1/6)

%%
run('zConstantGainLanding.m')
waitbar(2/6)

%%
run('zHorizontal.m')
waitbar(3/6)

%%
run('zTakeoff.m')
waitbar(4/6)

%%
run('zScaling.m')
waitbar(5/6)

%%
run('zFPS.m')
%%
waitbar(1.0)
close(hbar)