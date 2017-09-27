%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad
% close all;
clear all;clc
testName = 'ff_profile';
modelName = 'sim/';
subName = '/';
run('plotSettings.m')
if(~exist([ myset.path modelName testName]))
    mkdir([ myset.path modelName testName])
end

set(0,'defaultlinelinewidth',1.5)

% savePlots = 1;
savePlots = 0;
%% Plot the feed forward takeoff procedure
FF = 1;
t = 0:0.01:0.9;
F = max(2*FF-FF*(2/(0.9))*t,FF);

figure;
hold on
% title('Feed forward takeoff profile','Interpreter','latex');
xlabel('Time ($s$)', 'Interpreter', 'Latex');
ylabel('Normalized Thrust Force ($F_e$)', 'Interpreter', 'Latex');
plot(t,F);
grid on
h = legend('$F_{ff}$','Location','NorthEast');
set(h,'Interpreter','Latex');
set(h,'FontSize',11);
%% Plot & Save

if(savePlots)
    run('latexPlot.m');
end