%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad
% close all;
clear all;clc;

% This line defines what this settings file does runs and plots
% Feed forward takeoff and apply the algorithm in both axis
testName = 'Takeoff';
modelName = 'sim/';
subName = '/';
run('plotSettings.m')
if(~exist([ myset.path modelName testName]))
    mkdir([ myset.path modelName testName])
end
set(0,'defaultlinelinewidth',1.5)

% savePlots = 1;
savePlots = 0;
%% Run code

% Algorithm or Groundtruth
control.runHor = 1;
control.runVert = 1;

% Takeoff or start from hover
takeoff = 0;
takeoff = 1;

% Noise or no noise
noise = 0;
noise = 1;

% control scaling => NO effect on the quad
scalePowerX = 5;
scalePowerZ = 3;
scalePowerNoise = 3;

% Set the ranges
FPSRange =20;
range = 1;
%% (Initial) Conditions

% End time
t_end = 20;

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

for iGainLoop = 0:1
    
    % Set the I gains
    IgainHor = 0.5*iGainLoop;
    IgainVert = 0.25*iGainLoop;
        
    SimQuadPD();
    
    %% Plot & Save
    
    loop =1;
    for fps = FPSRange
        for scale = range
            scaling();
            
            %% X and gain
            figure
            hold on
            xlabel('Time ($s$)', 'Interpreter', 'Latex');
            ylabel('Position ($m$)', 'Interpreter', 'Latex');
            plot(time{loop},stateX{loop}(:,1));
            axis([0 t_end 0 2])
            yyaxis right
            ylabel('Gains', 'Interpreter', 'Latex');
            plot(covTime{loop},pstateX{loop});
            lh = legend('$x$','$K_p$','AutoUpdate','off');
            set(lh,'Interpreter','Latex');
            set(lh,'FontSize',11);
            grid on
            axis([0 t_end 0 3])
            subName = ['/xAxisGain_' num2str(scale,'%1.1f\n') 'scale_' num2str(fps,'%d\n') 'fps_' num2str(iGainLoop,'%d\n') 'Igain_'];
            subName = strrep(subName,'.','_');
            if(savePlots)
                run('latexPlot.m');
            end
            %% CovX
            figure
            hold on
            xlabel('Time ($s$)', 'Interpreter', 'Latex');
            ylabel('Ventral Flow ($1/s$)', 'Interpreter', 'Latex');
            plot(covTime{loop},divX{loop});
            axis([0 t_end -1.5 1.5])
            yyaxis right
            ylabel('Covariance ($rad/s$)', 'Interpreter', 'Latex');
            plot(covTime{loop},covdivX{loop});
            axis([0 t_end -0.007 0.001])
            lh = legend('Ventral Flow','Covariance','AutoUpdate','off','location','southeast');
            set(lh,'FontSize',11);
            set(lh,'Interpreter','Latex');
            grid on
            subName = ['/covX_' num2str(scale,'%1.1f\n') 'scale_' num2str(fps,'%d\n') 'fps_' num2str(iGainLoop,'%d\n') 'Igain_'];
            subName = strrep(subName,'.','_');
            if(savePlots)
                run('latexPlot.m');
            end
            %% Z and gain
            figure
            hold on
            xlabel('Time ($s$)', 'Interpreter', 'Latex');
            ylabel('Height ($m$)', 'Interpreter', 'Latex');
            plot(time{loop},stateX{loop}(:,2));
            axis([0 t_end 0 2])
            yyaxis right
            ylabel('Gains', 'Interpreter', 'Latex');
            plot(covTime{loop},pstateZ{loop});
            lh = legend('$z$','$K_p$','AutoUpdate','off');
            set(lh,'Interpreter','Latex');
            set(lh,'FontSize',11);
            grid on
            axis([0 t_end 0 12])
            subName = ['/zAxisGain_' num2str(scale,'%1.1f\n') 'scale_' num2str(fps,'%d\n') 'fps_' num2str(iGainLoop,'%d\n') 'Igain_'];
            subName = strrep(subName,'.','_');
            if(savePlots)
                run('latexPlot.m');
            end
            %% CovZ
            figure
            hold on
            xlabel('Time ($s$)', 'Interpreter', 'Latex');
            ylabel('Divergence ($1/s$)', 'Interpreter', 'Latex');
            plot(covTime{loop},divZ{loop});
            %         axis([0 t_end -1.5 1.5])
            yyaxis right
            ylabel('Covariance ($N/s$)', 'Interpreter', 'Latex');
            plot(covTime{loop},covdivZ{loop});
            axis([0 t_end -0.08 0.01])
            lh = legend('Divergence','Covariance','AutoUpdate','off','location','southeast');
            set(lh,'FontSize',11);
            set(lh,'Interpreter','Latex');
            grid on
            subName = ['/covZ_' num2str(scale,'%1.1f\n') 'scale_' num2str(fps,'%d\n') 'fps_' num2str(iGainLoop,'%d\n') 'Igain_'];
            subName = strrep(subName,'.','_');
            if(savePlots)
                run('latexPlot.m');
            end
        end
    end
end