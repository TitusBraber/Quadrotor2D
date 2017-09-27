%%% Titus Braber - Visual based stabilization of micro quadrotors
%%% 2D simulation of a quad

%% Make the symbolic model
model();

%% Loop the simulation
loop = 1;
tic
for fps = FPSRange
    fps
    for scale = range
        scale
        
        % Run Settings
        scaling();
        
        % Make Equations of Motion file
        qd_func = matlabFunction(subs(qd,symbolic,numeric{loop}),'vars',{funcInput},'File','Quad2D_eom');
        
        %% Simulink model
        
        % initial takeoff thrust
        Fff = control.thrustfactor*control.Fe;
        
        % Simulate
        warning off;
        %     tic
        simout = sim('Quad2D');
        %     toc
        
        %% Log data
        time{loop} = simoutnonlinQ.time;
        stateX{loop} = simoutnonlinQ.data;
        u{loop} = simoutnonlinU.data;
        tof{loop}= simoutnonlinOF.time;
        of{loop}= simoutnonlinOF.data;
        
        covTime{loop} = simoutControl.time;
        pstateX{loop} = simoutControl.data(:,1);
        divX{loop}    = simoutControl.data(:,2);
        covdivX{loop} = covPlotScale*simoutControl.data(:,3);
        timeX{loop}   = simoutControl.data(:,4);
        pstateZ{loop} = simoutControl.data(:,5);
        divZ{loop}    = simoutControl.data(:,6);
        covdivZ{loop} = covPlotScale*simoutControl.data(:,7);
        timeZ{loop}   = simoutControl.data(:,8);
        Pdes{loop}    = simoutControl.data(:,9);
        
        StartX{loop}     = find(diff(timeX{loop}), 1, 'first')+1;
        EndX{loop}       = find(diff(timeX{loop}), 1, 'last')+1;
        timeStartX{loop} = covTime{loop}(StartX{loop});
        timeEndX{loop}   = covTime{loop}(EndX{loop});
        if(isempty(timeEndX{loop}))
            triggerX(loop) = -1;
        else
            triggerX(loop) = timeEndX{loop};
        end
        
        StartZ{loop}     = find(diff(timeZ{loop}), 1, 'first')+1;
        EndZ{loop}       = find(diff(timeZ{loop}), 1, 'last')+1;
        timeStartZ{loop} = covTime{loop}(StartZ{loop});
        timeEndZ{loop}   = covTime{loop}(EndZ{loop});
        if(isempty(timeEndZ{loop}))
            triggerZ(loop) = -1;
        else
            triggerZ(loop) = timeEndZ{loop};
        end
        loop=loop+1;
    end
    toc
end
loop = loop-1;

%% Plots
defaultColorOrder = get(groot,'DefaultAxesColorOrder');

%% Make sure loop never equals 0
if(loop==0)
    loop = 1;
end