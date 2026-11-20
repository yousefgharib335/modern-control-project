function torque = furuta_swingup_torque(state,acceleration,p)
% Invert the provisional swing-up plant for the requested arm acceleration.
s = sin(state(2));
c = cos(state(2));
armRate = state(3);
pendulumRate = state(4);
Jp = p.J1cm+p.m1*p.l1^2;
difference = Jp-p.J1ax;
coupling = p.m1*p.L0*p.l1;
targetEnergy = p.m1*p.g*p.l1;
M11 = p.J0+p.m1*p.L0^2+Jp*s^2+p.J1ax*c^2;
M12 = coupling*c;
velocity0 = 2*difference*s*c*armRate*pendulumRate-coupling*s*pendulumRate^2;
pendulumForce = difference*s*c*armRate^2+targetEnergy*s-p.b1*pendulumRate;
pendulumAcceleration = (pendulumForce-M12*acceleration)/Jp;
torque = M11*acceleration+M12*pendulumAcceleration+velocity0+p.b0*armRate;
end
