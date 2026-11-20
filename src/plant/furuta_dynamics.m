function dx = furuta_dynamics(x, voltage, p)
% State: arm angle, pendulum angle from upright, and their angular rates.
angle = x(2);
armRate = x(3);
pendulumRate = x(4);
s = sin(angle);
c = cos(angle);

pivotInertia = p.J1cm + p.m1*p.l1^2;
inertiaDifference = pivotInertia - p.J1ax;
coupling = p.m1*p.L0*p.l1;
mass = [p.J0 + p.m1*p.L0^2 + pivotInertia*s^2 + p.J1ax*c^2, coupling*c;
    coupling*c, pivotInertia];

current = (voltage - p.Ke*p.N*armRate)/p.R;
torque = p.eta*p.N*p.Kt*current;
velocityTerms = [2*inertiaDifference*s*c*armRate*pendulumRate - coupling*s*pendulumRate^2;
    -inertiaDifference*s*c*armRate^2];
gravity = [0; -p.m1*p.g*p.l1*s];
friction = [p.b0*armRate; p.b1*pendulumRate];
acceleration = mass \ ([torque; 0] - velocityTerms - gravity - friction);
dx = [armRate; pendulumRate; acceleration];
end
