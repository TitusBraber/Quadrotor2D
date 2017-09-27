%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad

%% Parameter scaling
numeric{loop} = [
    0.42*(scale^3);
    0.517*scale;
    0.127*scale;
    (0.42*(scale^3))/12 * ((0.517*scale)^2 + (0.127*scale)^2);
    9.81];
control.theta.Kp = 32*(scale^scalePowerX);
control.theta.Kd = 4*(scale^scalePowerX);
control.y.Kp = 10*(scale^scalePowerZ);
control.y.Ki = 10*(scale^scalePowerZ);
control.y.Kd = 3*(scale^scalePowerZ);

% Set the width as a global variable so it can be used in simulink
control.w = double(numeric{loop}(2));

% Define the nummerical equilibrium thrust
control.Fe = double(numeric{loop}(1)*numeric{loop}(5));

% The noise on the actuators
% ActuatorNoise = 0.001*scale^(scalePowerZ*2)*noise;
ActuatorNoise = 0.001*scale^(scalePowerNoise*2)*noise;

%oiseSeed = [Motor1,Motor2,GYRO,VF,Div]
noiseSeed = [23341,346834,734234,0,12];
%% Scale gains

% Set the gains for the slowing down phase
control.slowdownGainX = slowdownGainX;
control.slowdownGainZ = slowdownGainZ*scale^scalePowerZ;

% Set the desired starting Gains
control.startGainX = startGainX;
control.startGainZ = startGainZ*scale^scalePowerZ;

% Increasing gain for oscillations and their limits
control.gainHor = gainHor;
control.gainVert = gainVert*scale^scalePowerZ;

% Set the I gains
control.IgainVert = IgainVert*scale^scalePowerZ;
control.IgainHor = IgainHor;
%% FPS scaler
control.window = round(windowbase*20/fps);
control.delay = round(delaybase*20/fps);
control.covLimitVert = covLimitVertBase*(scale^3);
