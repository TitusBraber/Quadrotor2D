%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad
% close all;
clear all;clc;

% This line defines what this settings file does runs and plots
% FPS related tests
testName = 'FPS';
modelName = 'sim/';
subName = '/';
run('plotSettings.m')
if(~exist([ myset.path modelName testName]))
    mkdir([ myset.path modelName testName])
end
set(0,'defaultlinelinewidth',1.5)

savePlots = 1;
savePlots = 0;
%% Run code

% Algorithm or Groundtruth
control.runHor = 1;
control.runVert = 1;

% Takeoff or start from hover
takeoff = 0;
% takeoff = 1;

% Noise or no noise
noise = 0;
noise = 1;

% Only dynamics scaling, no control scaling => Effect on the quad
% scalePowerX = 3;
% scalePowerZ = 2;
% scalePowerNoise = 2;

% control scaling => NO effect on the quad
scalePowerX = 5;
scalePowerZ = 3;
scalePowerNoise = 2;

FPSscaler = 1;
FPSscaler = [1:-0.05:0.7];

% Set the ranges
FPSRange =round(20*FPSscaler);
range = 1;
%% (Initial) Conditions

% End time
t_end = 30;

% End the simulation if the algorithm triggers
triggerStopX = 0;
triggerStopZ = 0;

if(takeoff)
    % Takeoff
    % Start with FFstart seconds thrust
    control.thrustfactor = 2;
    control.FFstart = 0.6;
    q0 = [1,0.1,degtorad(0),0,0,0]'; % Start at 0.1 meter so we don't /0
else
    % From Hover, but give it a slight push to make sure div !=0
    control.thrustfactor = 1.1;
    control.FFstart = 0.1;
    q0 = [1,1,degtorad(0),0,0,0]';
end

control.setpoint = [1,1,0,0,0,0];

%% Set gains and limits
% N.B. THEY ARE POSSIBLY SCALED IN scaling()!

% Set the gains during slowdown
slowdownGainX = 0;
slowdownGainZ = 10;

% Set the desired starting Gains.
startGainX = 0;
startGainZ = 0;

% Increasing gain per second
gainHor = 0.3; 
gainVert = 2;

% Set the I gains
IgainHor = 0.5;
IgainVert = 0.25;

% Set the stability fractions
control.stableFractionHor = 0.6;
control.stableFractionVert = 0.6;

% Set the cov limits
control.covLimitHor = -6.0e-3;
covLimitVertBase = -4.5e-2;
%% Less interesting settings

% Set the slowed down DivY
control.SlowedDownDivZ = 0.5;

control.restartHor = 0;
control.restartVert = 0;

% The noise on the vision
visionNoise = 1/30000*noise;

% IMU specifications
GyroNoise = (0.005^2)*noise; % True noise value from datasheet.
GyroFreq = 1/80;

% Set the window sizes
windowbase = 30;
delaybase = 15;

% Set the desired divergence
control.divHor = 0;
control.divVert = 0;

% Slowmotion animation
slowmo =1/1;

covPlotScale = 1;
%% Run simulation

SimQuadPD();

%% Multiple X/Y axes plots
close all;
start = 1;
efFPS = FPSRange(start:end);
set(groot,'DefaultAxesColorOrder',parula(length(efFPS)));

% DistanceX
loop = start;
figure
hold on
xlabel('Time ($s$)', 'Interpreter', 'Latex');
ylabel('Position ($m$)', 'Interpreter', 'Latex');
for fps = efFPS
    plot(time{loop},stateX{loop}(:,1));
    loop = loop+1;
    %     lh = legend(cellstr(num2str(efFPS', 'fps = %d')),'location','southwest');
end
lh = colorbar('Direction','reverse');
caxis([min(efFPS) max(efFPS)])
ylabel(lh, 'FPS')
set(lh,'Ticks',wrev(efFPS),'TickLabels',{efFPS});
grid on
subName = ['/XmaxFPS_' num2str(scale,'%d\n') 'scale_'];
if(savePlots)
    run('latexPlot.m');
end

% DistanceZ
loop = start;
figure
hold on
xlabel('Time ($s$)', 'Interpreter', 'Latex');
ylabel('Height ($m$)', 'Interpreter', 'Latex');
for fps = efFPS
    plot(time{loop},stateX{loop}(:,2));
    loop = loop+1;
    %     lh = legend(cellstr(num2str(efFPS', 'fps = %d')),'location','southwest');
end
lh = colorbar('Direction','reverse');
caxis([min(efFPS) max(efFPS)])
ylabel(lh, 'FPS')
set(lh,'Ticks',wrev(efFPS),'TickLabels',{efFPS});
grid on
subName = ['/ZmaxFPS_' num2str(scale,'%d\n') 'scale_'];
if(savePlots)
    run('latexPlot.m');
end

loop = loop -1;
set(groot,'DefaultAxesColorOrder',defaultColorOrder);

%%
% FPSscaler = 2;
FPSscaler = [5:-0.4:1];
% Set the ranges
noiseRange = 0;
FPSRange =round(20*FPSscaler);%:-5:5;%0.2;%0.2;%(10:-3:1)/30; %  6/30;%(5:-1:0)/100;%
range = 0.2;%[(10:-1:1)/10];%[0.4:-0.1:0.1];%[1:-0.1:0.1];[0.3,0.4];%[1 0.5 0.2 0.1 0.05 0.01];%[


visionNoises = [0 1/30000*noise];

for noisevision = 0:1
    visionNoise = visionNoises(noisevision+1);
    
    SimQuadPD();
    
    % Multiple X/Y axes plots
    start = 1;
    efFPS = FPSRange(start:end);
    set(groot,'DefaultAxesColorOrder',parula(length(efFPS)));
    
    % DistanceX
    loop = start;
    figure
    hold on
    xlabel('Time ($s$)', 'Interpreter', 'Latex');
    ylabel('Position ($m$)', 'Interpreter', 'Latex');
    for fps = efFPS
        plot(time{loop},stateX{loop}(:,1));
        loop = loop+1;
    end
    lh = colorbar('Direction','reverse');
    % set(lh,'Limits',[min(efFPS) max(efFPS)]);
    caxis([min(efFPS) max(efFPS)])
    ylabel(lh, 'FPS')
    set(lh,'Ticks',wrev(efFPS),'TickLabels',{efFPS});
    grid on
    subName = ['/XmaxFPS_' num2str(scale,'%1.1f\n') 'scale_' num2str(noisevision,'%d\n') 'noise_'];
    subName = strrep(subName,'.','_');
    if(savePlots)
        run('latexPlot.m');
    end
    
    % DistanceZ
    loop = start;
    figure
    hold on
    xlabel('Time ($s$)', 'Interpreter', 'Latex');
    ylabel('Height ($m$)', 'Interpreter', 'Latex');
    for fps = efFPS
        plot(time{loop},stateX{loop}(:,2));
        loop = loop+1;
    end
    lh = colorbar('Direction','reverse');
    % set(lh,'Limits',[min(efFPS) max(efFPS)]);
    caxis([min(efFPS) max(efFPS)])
    ylabel(lh, 'FPS')
    set(lh,'Ticks',wrev(efFPS),'TickLabels',{efFPS});
    grid on
    axis([0 t_end 0.8 1.6]);
    subName = ['/ZmaxFPS_' num2str(scale,'%1.1f\n') 'scale_' num2str(noisevision,'%d\n') 'noise_'];
    subName = strrep(subName,'.','_');
    if(savePlots)
        run('latexPlot.m');
    end
    
    loop = loop -1;
    set(groot,'DefaultAxesColorOrder',defaultColorOrder);
end