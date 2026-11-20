function [voltage,energy,requestedVoltage] = furuta_swingup(state,p,cfg)
% Energy shaping produces a desired arm acceleration, mapped to motor voltage.
arm = state(1); pendulum = state(2);
armRate = state(3); pendulumRate = state(4);
c = cos(pendulum);
Jp = p.J1cm+p.m1*p.l1^2;
targetEnergy = p.m1*p.g*p.l1;
energy = 0.5*Jp*pendulumRate^2+targetEnergy*c;
acceleration = cfg.energyGain*(energy-targetEnergy-cfg.energyOffset)* ...
    tanh(cfg.signSlope*pendulumRate*c) ...
    - cfg.armCenterGain*arm-cfg.armDampingGain*armRate;
% Supply an initial push when exactly at rest near the hanging equilibrium.
if abs(pendulumRate)<cfg.kickRate && energy<0
    acceleration = acceleration+cfg.kickAcceleration;
end
acceleration = min(cfg.accelerationLimit,max(-cfg.accelerationLimit,acceleration));
torque = furuta_swingup_torque(state,acceleration,p);
motorGain = p.eta*p.N*p.Kt/p.R;
requestedVoltage = torque/motorGain+p.Ke*p.N*armRate;
voltage = min(cfg.voltageLimit,max(-cfg.voltageLimit,requestedVoltage));
end
