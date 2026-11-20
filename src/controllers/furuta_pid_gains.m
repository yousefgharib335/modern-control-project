function gains = furuta_pid_gains(p)
% Initial PID/PD gains synthesized from the upright model, then checked nonlinearly.
[A,B] = furuta_linearize(p);
augmentedA = [A zeros(4,1); 1 0 0 0 0];
augmentedB = [B;0];
controllability = [augmentedB, augmentedA*augmentedB, ...
    augmentedA^2*augmentedB, augmentedA^3*augmentedB, augmentedA^4*augmentedB];
assert(rank(controllability)==5,'FurutaPID:Controllability','Augmented plant is uncontrollable.');
targetPoles = [-1.5 -2 -3 -4 -5];
coefficients = poly(targetPoles);
polynomial = eye(5);
for k = 2:6
    polynomial = polynomial*augmentedA + coefficients(k)*eye(5);
end
% Ackermann's formula gives u = -K*[x; integral(theta0)].
feedback = -([0 0 0 0 1]/controllability)*polynomial;
gains = struct('armP',feedback(1),'pendulumP',feedback(2), ...
    'armD',feedback(3),'pendulumD',feedback(4),'armI',feedback(5), ...
    'filterRate',80,'antiWindupRate',5,'voltageLimit',12, ...
    'initialState',[0;deg2rad(5);0;0],'targetPoles',targetPoles);
end
