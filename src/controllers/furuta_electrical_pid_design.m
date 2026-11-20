function [g,poles] = furuta_electrical_pid_design(data,compensation)
% Design the filtered PID for the compensated cycle-average voltage input.
A = data.upright.A;
B = data.designB;
g = struct('armP',0,'pendulumP',0,'armD',0,'pendulumD',0,'armI',0, ...
    'filterRate',80,'antiWindupRate',5,'voltageLimit',compensation.equivalentVoltageLimit, ...
    'initialState',[0;deg2rad(1);0;0],'targetPoles',[-2 -2.75 -3.5 -4.25 -5]);
gainNames = {'armP','pendulumP','armD','pendulumD','armI'};
gainEquation = zeros(5);
for k = 1:5
    s = g.targetPoles(k);
    angleResponse = (s*eye(4)-A)\B;
    angleResponse = angleResponse(1:2).';
    filteredRate = g.filterRate*s/(s+g.filterRate);
    gainEquation(k,:) = [angleResponse filteredRate*angleResponse angleResponse(1)/s];
end
values = gainEquation\ones(5,1);
for k = 1:5
    g.(gainNames{k}) = values(k);
end
linear = zeros(7);
linear(1:4,1:4) = A;
linear(5,1) = g.armI;
linear(6:7,1:2) = g.filterRate*eye(2);
linear(6:7,6:7) = -g.filterRate*eye(2);
feedback = [g.armP+g.armD*g.filterRate, g.pendulumP+g.pendulumD*g.filterRate, ...
    0,0,1,-g.armD*g.filterRate,-g.pendulumD*g.filterRate];
poles = eig(linear+[B;zeros(3,1)]*feedback);
assert(all(real(poles)<0) && all(min(abs(poles-g.targetPoles),[],1)<1e-4), ...
    'Average-design PID poles did not match their targets.');
end
