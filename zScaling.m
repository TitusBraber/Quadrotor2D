%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad
% close all;
clear all;clc;

% This line defines what this settings file does runs and plots
% Scaling information
testName = 'Scaling';
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
% takeoff = 1;

% Noise or no noise
noise = 0;
noise = 1;

% Only dynamics scaling, no control scaling => Effect on the quad
scalePowerX = 3;
scalePowerZ = 2;
scalePowerNoise = 2; % Make sure noise does have a higher influence.

% control scaling => NO effect on the quad
% scalePowerX = 5;
% scalePowerZ = 3;
% scalePowerNoise = 2; % Make sure noise does have a higher influence.

FPSscaler = 1;

% Set the ranges
FPSRange =20*FPSscaler;
range = (10:-1:1)/10;
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

%% Run and plot
for compensation = 1:3
    
    if(compensation == 1)
        % Only dynamics scaling, no control scaling => Effect on the quad
        scalePowerX = 3;
        scalePowerZ = 2;
        scalePowerNoise = 2;
    elseif(compensation == 2)
        % control scaling => NO effect on the quad
        scalePowerX = 5;
        scalePowerZ = 3;
        scalePowerNoise = 3;
    else
        % control scaling => NO effect on the quad
        scalePowerX = 5;
        scalePowerZ = 3;
        scalePowerNoise = 2; % Make sure noise does have a higher influence.
    end
    
    SimQuadPD();
    
    %% Plot & Save
    set(groot,'DefaultAxesColorOrder',parula(length(range)));
    
    % DistanceX
    loop = 1;
    for fps = FPSRange
        figure
        hold on
        xlabel('Time ($s$)', 'Interpreter', 'Latex');
        ylabel('Position ($m$)', 'Interpreter', 'Latex');
        for scale = range
            %             plotReport(time{loop},stateX{loop}(:,1),loop);
            plot(time{loop},stateX{loop}(:,1));
            loop = loop+1;
        end
        lh = colorbar( 'Direction','reverse');
        set(lh,'Limits',[0.1 1]);
        ylabel(lh, 'Scale')
        set(lh,'Ticks',wrev(range),'TickLabels',{range});
        axis([0 t_end 0.8 1.25])
        grid on
        subName = ['/distanceX_' num2str(fps,'%d\n') 'fps_' num2str(compensation-1,'%d\n') 'comp_'];
        
        if(savePlots)
            run('latexPlot.m');
        end
    end
    
    % DistanceZ
    loop = 1;
    for fps = FPSRange
        figure
        hold on
        xlabel('Time ($s$)', 'Interpreter', 'Latex');
        ylabel('Height ($m$)', 'Interpreter', 'Latex');
        for scale = range
            %             plotReport(time{loop},stateX{loop}(:,2),loop);
            plot(time{loop},stateX{loop}(:,2));
            loop = loop+1;
        end
        lh = colorbar( 'Direction','reverse');
        set(lh,'Limits',[0.1 1]);
        ylabel(lh, 'Scale')
        set(lh,'Ticks',wrev(range),'TickLabels',{range});
        %         axis([0 t_end -0.5 3.5])
        %         axis([0 t_end 0 2])
        grid on
        subName = ['/distanceZ_' num2str(fps,'%d\n') 'fps_' num2str(compensation-1,'%d\n') 'comp_'];
        
        if(savePlots)
            run('latexPlot.m');
        end
    end
    %%
    % GainTimeX
    loop =1;
    for fps = FPSRange
        figure
        hold on
        xlabel('Time ($s$)', 'Interpreter', 'Latex');
        ylabel('Control Gain', 'Interpreter', 'Latex');
        for scale = range
            %             title(['Horizontal X axis: ' num2str(fps,'%1.3f\n') ' (fps)']);
            plot(covTime{loop},pstateX{loop});
            loop=loop+1;
        end
        lh = colorbar( 'Direction','reverse');
        set(lh,'Limits',[0.1 1]);
        ylabel(lh, 'Scale')
        set(lh,'Ticks',wrev(range),'TickLabels',{range});
        grid on
        subName = ['/gainTimeX_' num2str(fps,'%d\n') 'fps_' num2str(compensation-1,'%d\n') 'comp_'];
        if(savePlots)
            run('latexPlot.m');
        end
    end
    
    
    % GainTimeZ
    loop =1;
    for fps = FPSRange
        figure
        hold on
        xlabel('Time ($s$)', 'Interpreter', 'Latex');
        ylabel('Control Gain', 'Interpreter', 'Latex');
        for scale = range
            %             title(['Vertical Z axis: ' num2str(fps,'%1.3f\n') ' (fps)']);
            plot(covTime{loop},pstateZ{loop});
            loop=loop+1;
        end
        lh = colorbar( 'Direction','reverse');
        set(lh,'Limits',[0.1 1]);
        ylabel(lh, 'Scale')
        set(lh,'Ticks',wrev(range),'TickLabels',{range});
        grid on
        subName = ['/gainTimeZ_' num2str(fps,'%d\n') 'fps_' num2str(compensation-1,'%d\n') 'comp_'];
        if(savePlots)
            run('latexPlot.m');
        end
    end
    
    
    
    set(groot,'DefaultAxesColorOrder',defaultColorOrder);
    
end

set(groot,'DefaultAxesColorOrder',parula(length(range)));
loop = 1;
for fps = FPSRange
    figure
    hold on
    xlabel('Time ($s$)', 'Interpreter', 'Latex');
    ylabel('Height ($m$)', 'Interpreter', 'Latex');
    for scale = range(1:end-1)
        %             plotReport(time{loop},stateX{loop}(:,2),loop);
        plot(time{loop},stateX{loop}(:,2));
        loop = loop+1;
    end
    %     h = findobj(gca,'Type','line');
    %     lh = legend([h(10) h(1)],{num2str([1 0.1]', 'Scale = %1.1f')});
    %     lh = legend(cellstr(num2str(range', 'Scale = %1.1f')));
    
    lh = colorbar( 'Direction','reverse');
    set(lh,'Limits',[0.1 1]);
    ylabel(lh, 'Scale')
    set(lh,'Ticks',wrev(range),'TickLabels',{range});
    %     axis([0 t_end -0.5 3.5])
    grid on
    subName = ['/distanceZ_' num2str(fps,'%d\n') 'fps_' num2str(compensation-1,'%d\n') 'comp_zoomed_'];
    
    if(savePlots)
        run('latexPlot.m');
    end
end

set(groot,'DefaultAxesColorOrder',defaultColorOrder);
%%
% triggerScaleX
        loop = 1;
        figure
        for fps = FPSRange
            hold on
            %             title('Horizontal Gain triggered in Scale vs Time');
            xlabel('Scale', 'Interpreter', 'Latex');
            ylabel('Time ($s$)', 'Interpreter', 'Latex');
            plot(range,triggerX(1+(loop-1)*length(range):loop*length(range)),'*');
            loop = loop+1;
            lh = legend('Trigger','location','southeast');
            set(lh,'FontSize',11);
            set(lh,'Interpreter','Latex');
            grid on
        end
        subName = ['/triggerScaleX_'];
        if(savePlots)
            run('latexPlot.m');
        end
%% 
% triggerScaleZ
loop = 1;
figure
for fps = FPSRange
    hold on
    %             title('Horizontal Gain triggered in Scale vs Time');
    xlabel('Scale', 'Interpreter', 'Latex');
    ylabel('Time ($s$)', 'Interpreter', 'Latex');
    plot(range,triggerZ(1+(loop-1)*length(range):loop*length(range)),'*');
    loop = loop+1;
    lh = legend('Trigger','location','southeast');
    set(lh,'FontSize',11);
    set(lh,'Interpreter','Latex');
    grid on
end
subName = ['/triggerScaleZ_'];
if(savePlots)
    run('latexPlot.m');
end
