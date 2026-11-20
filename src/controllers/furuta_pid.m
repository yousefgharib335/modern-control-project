function [voltage, requestedVoltage, stateDerivative] = furuta_pid(theta0,theta1,state,g)
% Continuous PID/PD controller. State = [integral voltage; filtered angles].
% Initialize state with [0; initial theta0; initial theta1] for each experiment.
armRate = g.filterRate*(theta0-state(2));
pendulumRate = g.filterRate*(theta1-state(3));
requestedVoltage = g.armP*theta0 + state(1) + g.armD*armRate + ...
    g.pendulumP*theta1 + g.pendulumD*pendulumRate;
voltage = min(g.voltageLimit,max(-g.voltageLimit,requestedVoltage));
stateDerivative = [g.armI*theta0 + g.antiWindupRate*(voltage-requestedVoltage); ...
    armRate;pendulumRate];
end
