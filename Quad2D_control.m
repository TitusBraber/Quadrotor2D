%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad

%% Equations of motion
function out = Quad2D_control(u)
%% Declarations
global control;
persistent F FFstartX FFstartZ pstateX pstateZ stateX stateZ prevtimeX prevtimeZ err_sumZ err_sumX cov_divX pdesired_history pdesired cov_divZ ind_histX ind_histZ divergence_history thrust_history flow_history oscX oscZ;
if( isempty(FFstartX) || isempty(FFstartZ) )
    F=0;
    pdesired = 0;
    FFstartZ = 0;
    pstateZ = 0;
    prevtimeZ = 0;
    err_sumZ = 0;
    cov_divZ = 0;
    ind_histZ = 0;
    thrust_history = zeros(1,control.window);
    divergence_history = zeros(1,control.window);
    oscZ = 0;
    stateZ = 0;
    
    FFstartX = 0;
    cov_divX = 0;
    ind_histX = 0;
    err_sumX = 0;
    oscX = 0;
    stateX = 0;
    pstateX = 0;
    prevtimeX = 0;
    flow_history = zeros(1,control.window);
    pdesired_history = zeros(1,control.window);
end

%% Inputs
x  = u(1);
y = u(2);
p = u(3);
xd = u(4);
yd = u(5);
pd = u(6);
divX = u(7);
divZ = u(8);
FF = control.Fe;
maxBank = 2*0.3491;
time  = get_param('Quad2D','SimulationTime');

if(~control.runHor)
    %% Ground Truth x control
    pstateX = 1;
    tiltXD = max(-maxBank, min(-xd, maxBank));
    tiltX= max(-maxBank, min((control.setpoint(1)-x), maxBank));
    pdesired = tiltX + tiltXD;
else
    %% Control loop
    if(time>control.FFstart && FFstartX)
        dtX = time-prevtimeX;
        
        % Log Flow and history covariance
        flow_history(mod(ind_histX,control.window)+1) = divX;
        pdesired_history(mod(ind_histX,control.window)+1) = pdesired;
        ind_histX = ind_histX+1;
        
        if (ind_histX >= 2*control.window)
            n_elements = length(flow_history);
            sumX = 0;
            sumY = 0;
            sumXY = 0;
            
            for i = 1:n_elements
                sumX = sumX + pdesired_history(i);
                sumY = sumY+ flow_history(i);
                sumXY = sumXY + pdesired_history(i) * flow_history(i);
            end
            cov_divX = (sumXY / n_elements - sumX * sumY / (n_elements * n_elements));
        else
            cov_divX = 0;
        end
        
        %% Check if oscillating
        if(cov_divX < control.covLimitHor)
            if(oscX==0)
                oscX = 1;
                stateX = 3;
                pstateX = pstateX*control.stableFractionHor;
            end
        elseif(control.restartHor)
            % Restart algorithm
            oscX = 0;
        end
        
        %% Adept gains
        errX = control.divHor - divX;
        
        if(oscX==0)
            pstateX = pstateX + control.gainHor*dtX;
        end
        
        err_sumX = err_sumX+(errX*dtX);
        
        OF = pstateX*errX + control.IgainHor*err_sumX;
        
        pdesired = max(-maxBank, min(OF, maxBank));
    elseif(time>control.FFstart)
        %% Slow down from FFstart
        pstateX = control.slowdownGainX;
        
        if(divZ<control.SlowedDownDivZ)
            FFstartX = 1;
            stateX = 1;
            pstateX = control.startGainX;
        end
        
        OF = - pstateX * divX;
        pdesired = max(-maxBank, min(OF, maxBank));
    else
        pdesired = 0;
    end
    prevtimeX = time;
end


%% Ground Truth Z control
if(~control.runVert)
    %     keyboard
    if(time>=control.FFstart)
        pstateZ = control.y.Kp;
        
        M = min(FF*control.w/2,max(-FF*control.w/2,control.theta.Kp*(pdesired-p) - control.theta.Kd*pd));
        F = min((2*FF-2*abs(M)/control.w),max(2*abs(M)/control.w,FF/(cos(p)) + pstateZ*(control.setpoint(2)-y) - control.y.Kd*yd));
        
        F1 = F/2+(M/control.w);
        F2 = F/2-(M/control.w);
    else
        M = min(FF*control.w/2,max(-FF*control.w/2,control.theta.Kp*(pdesired-p) - control.theta.Kd*pd));
        F = min((2*FF-2*abs(M)/control.w),max(2*abs(M)/control.w,max(control.thrustfactor*FF-FF*(2/(control.FFstart))*time,control.Fe)));
        
        F1 = F/2+(M/control.w);
        F2 = F/2-(M/control.w);
    end
else
    %% Control loop
    if(time>=control.FFstart && FFstartZ)
        dtZ = time-prevtimeZ;
        
        %% Log Divergence and history covariance
        divergence_history(mod(ind_histZ,control.window)+1) = divZ;
        thrust_history(mod(ind_histZ,control.window)+1) = F;
        ind_histZ = ind_histZ+1;
        
        if (ind_histZ >= 2*control.window)
            n_elements = length(divergence_history);
            sumY = 0;
            sumX = 0;
            sumXY = 0;
            
            for i = 1:n_elements
                sumX = sumX + thrust_history(i);
                sumY = sumY+ divergence_history(i);
                sumXY = sumXY + thrust_history(i) * divergence_history(i);
            end
            cov_divZ = (sumXY / n_elements - sumX * sumY / (n_elements * n_elements));
        else
            cov_divZ = 0;
        end
        %% Check if oscZillating
        if(cov_divZ < control.covLimitVert)
            if(oscZ==0)
                oscZ = 1;
                stateZ = 3;
                pstateZ = pstateZ*control.stableFractionVert;
            end
        elseif(control.restartVert)
            % Restart algorithm
            oscZ = 0;
        end
        
        %% Adept gains
        errZ = control.divVert - divZ;
        if(oscZ==0)
            pstateZ = pstateZ + control.gainVert*dtZ;
        end
        err_sumZ = err_sumZ+(errZ*dtZ);
        
        %% Determine control
        M = min(FF*control.w/2,max(-FF*control.w/2,control.theta.Kp*(pdesired-p) - control.theta.Kd*pd));
        F = min((2*FF-2*abs(M)/control.w),max(2*abs(M)/control.w,FF/(cos(p)) + pstateZ*errZ + control.IgainVert*err_sumZ));
        
        F1 = F/2+(M/control.w);
        F2 = F/2-(M/control.w);
    elseif(time>=control.FFstart)
        %% Slow down from FFstart
        pstateZ = control.slowdownGainZ;
        if(divZ<control.SlowedDownDivZ)
            FFstartZ = 1;
            stateZ = 1;
            pstateZ = control.startGainZ;
        end
        
        M = min(FF*control.w/2,max(-FF*control.w/2,control.theta.Kp*(pdesired-p) - control.theta.Kd*pd));
        F = min((2*FF-2*abs(M)/control.w),max(2*abs(M)/control.w,min(2*FF,max(0,FF/(cos(p)) - pstateZ*divZ))));
        
        F1 = max(FF/4,F/2+(M/control.w));
        F2 = max(FF/4,F/2-(M/control.w));
    else
        M = min(FF*control.w/2,max(-FF*control.w/2,control.theta.Kp*(pdesired-p) - control.theta.Kd*pd));
        F = min((2*FF-2*abs(M)/control.w),max(2*abs(M)/control.w,max(control.thrustfactor*FF-FF*(2/(control.FFstart))*time,control.Fe)));
        
        F1 = F/2+(M/control.w);
        F2 = F/2-(M/control.w);
    end
    prevtimeZ = time;
end

out = [F1;F2;pstateX;divX;cov_divX;stateX;pstateZ;divZ;cov_divZ;stateZ;pdesired];

end
