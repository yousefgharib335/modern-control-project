function design = furuta_pole_placement_design(A,B,C,D,targetPoles)
% Place the four upright plant poles with full-state feedback u = -K*x.
% The single-argument form takes the original physical parameter struct.
if nargin==1
    [A,B,C,D] = furuta_linearize(A);
end
reachability = [B A*B A^2*B A^3*B];
assert(rank(reachability)==4,'FurutaPolePlacement:Controllability', ...
    'The upright model must be controllable.');
if nargin<5
    targetPoles = [-2 -3 -4 -5];
end
coefficients = poly(targetPoles);
polynomial = eye(4);
for k = 2:5
    polynomial = polynomial*A + coefficients(k)*eye(4);
end
K = ([0 0 0 1]/reachability)*polynomial;
actualPoles = sort(eig(A-B*K));
assert(max(abs(actualPoles-sort(targetPoles(:))))<1e-6, ...
    'FurutaPolePlacement:Poles','The achieved poles differ from the targets.');
design = struct('A',A,'B',B,'C',C,'D',D,'K',K, ...
    'targetPoles',targetPoles,'actualPoles',actualPoles, ...
    'voltageLimit',12,'initialState',[0;deg2rad(5);0;0]);
end
