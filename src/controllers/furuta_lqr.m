function [voltage,requestedVoltage] = furuta_lqr(state,K,voltageLimit)
% State order: theta0, theta1, omega0, omega1 in radians and radians/second.
requestedVoltage = -K*state;
voltage = min(voltageLimit,max(-voltageLimit,requestedVoltage));
end
