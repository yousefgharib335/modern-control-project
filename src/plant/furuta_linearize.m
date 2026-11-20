function [A,B,C,D] = furuta_linearize(p, equilibriumAngle)
% Linear model at theta1 = 0 (upright) or pi (hanging), with zero speed and input.
if nargin < 2
    equilibriumAngle = 0;
end
assert(ismember(equilibriumAngle,[0 pi]), ...
    'FurutaPlant:NotEquilibrium','Use 0 for upright or pi for hanging.');
c = cos(equilibriumAngle);
coupling = p.m1*p.L0*p.l1*c;
mass = [p.J0 + p.m1*p.L0^2 + p.J1ax, coupling;
    coupling, p.J1cm + p.m1*p.l1^2];
gravitySlope = [0 0; 0 p.m1*p.g*p.l1*c];
damping = diag([p.b0 + p.eta*p.N^2*p.Kt*p.Ke/p.R, p.b1]);
A = [zeros(2), eye(2); mass \ gravitySlope, -(mass \ damping)];
B = [zeros(2,1); mass \ [p.eta*p.N*p.Kt/p.R; 0]];
C = [eye(2), zeros(2)];
D = zeros(2,1);
end
