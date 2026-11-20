function design = furuta_lqr_design(A,B,C,D,voltageScale)
% Continuous-time LQR for upright full-state regulation, u = -K*x.
% The single-argument form takes the original physical parameter struct.
if nargin==1
    [A,B,C,D] = furuta_linearize(A);
end
if nargin<5
    voltageScale = 6;
end
reachability = [B A*B A^2*B A^3*B];
assert(rank(reachability)==4,'FurutaLQR:Controllability', ...
    'The upright model must be controllable.');

% Bryson scaling: 20 deg arm, 5 deg pendulum, and 2 rad/s rates.
stateScales = [deg2rad(20);deg2rad(5);2;2];
Q = diag(1./stateScales.^2);
R = 1/voltageScale^2;

% Solve CARE from the stable invariant subspace of its Hamiltonian matrix.
% This small dense solve also works without Control System Toolbox.
hamiltonian = [A -(B/R)*B'; -Q -A'];
[vectors,values] = eig(hamiltonian);
stable = real(diag(values))<0;
assert(nnz(stable)==4,'FurutaLQR:Riccati','Expected four stable Hamiltonian modes.');
upper = vectors(1:4,stable);
lower = vectors(5:8,stable);
assert(rcond(upper)>1e-12,'FurutaLQR:Conditioning','Riccati invariant subspace is ill-conditioned.');
S = lower/upper;
assert(norm(imag(S),'fro')<1e-8*max(1,norm(S,'fro')), ...
    'FurutaLQR:Riccati','Riccati solution has significant imaginary components.');
S = real((S+S')/2);
K = (B'*S)/R;
residual = A'*S+S*A-S*B*K+Q;
residualScale = 1+norm(A'*S,'fro')+norm(S*A,'fro')+norm(S*B*K,'fro')+norm(Q,'fro');
assert(norm(residual,'fro')/residualScale<1e-9 && min(eig(S))>0, ...
    'FurutaLQR:Riccati','Invalid positive-definite Riccati solution.');
actualPoles = eig(A-B*K);
assert(all(real(actualPoles)<0),'FurutaLQR:Stability','LQR closed loop is unstable.');
design = struct('A',A,'B',B,'C',C,'D',D,'K',K,'Q',Q,'R',R,'S',S, ...
    'stateScales',stateScales,'voltageScale',voltageScale, ...
    'riccatiResidual',norm(residual,'fro')/residualScale, ...
    'actualPoles',actualPoles,'voltageLimit',12,'initialState',[0;deg2rad(5);0;0]);
end
