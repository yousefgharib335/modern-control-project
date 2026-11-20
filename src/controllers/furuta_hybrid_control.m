function [voltage,energy,angle,requestedVoltage] = furuta_hybrid_control(state,mode,armReference,p,cfg,K)
% Mode 1 swings up, mode 2 stabilizes, mode 0 commands zero motor voltage.
angle = atan2(sin(state(2)),cos(state(2)));
Jp = p.J1cm+p.m1*p.l1^2;
energy = 0.5*Jp*state(4)^2+p.m1*p.g*p.l1*cos(state(2));
voltage = 0;
requestedVoltage = 0;
if mode==1
    [voltage,energy,requestedVoltage] = furuta_swingup(state,p,cfg);
elseif mode==2
    errorState = [state(1)-armReference;angle;state(3);state(4)];
    [voltage,requestedVoltage] = furuta_lqr(errorState,K,cfg.voltageLimit);
end
end
