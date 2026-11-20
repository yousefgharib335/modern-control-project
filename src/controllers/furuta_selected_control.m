function [voltage,energy,angle,requestedVoltage,nextMemory] = furuta_selected_control(state,mode,armReference,p,cfg,controllers,controllerId,memory,swing)
% Select one stabilization law. Memory contains PID states and previous mode/ID.
if nargin==9 && mode==1
    voltage = swing(1);
    energy = swing(2);
    requestedVoltage = swing(3);
    angle = atan2(sin(state(2)),cos(state(2)));
else
    [voltage,energy,angle,requestedVoltage] = furuta_hybrid_control( ...
        state,mode,armReference,p,cfg,controllers.lqrK);
end
nextMemory = [0;0;0;mode;controllerId];
if mode~=2
    return
end
errorState = [state(1)-armReference;angle;state(3);state(4)];
if controllerId==1
    g = controllers.pid;
    pidState = memory(1:3);
    if memory(4)~=2 || memory(5)~=1
        % Initialize derivatives from the measured rates on entering PID.
        pidState = [0;errorState(1)-state(3)/g.filterRate;angle-state(4)/g.filterRate];
    end
    [voltage,requestedVoltage,derivative] = furuta_pid(errorState(1),angle,pidState,g);
    % Exact low-pass updates and Euler integration of the anti-windup state.
    decay = exp(-g.filterRate*cfg.sampleTime);
    nextMemory(1) = pidState(1)+cfg.sampleTime*derivative(1);
    nextMemory(2:3) = decay*pidState(2:3)+(1-decay)*errorState(1:2);
elseif controllerId==2
    [voltage,requestedVoltage] = furuta_pole_placement(errorState,controllers.poleK,cfg.voltageLimit);
elseif controllerId~=3
    voltage = 0;
    requestedVoltage = 0;
end
end
