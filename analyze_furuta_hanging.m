function results = analyze_furuta_hanging
% Linearize the CAD torque-input model at the hanging equilibrium.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
nominal = furuta_cad_parameters;
nominal.armDamping = 0;
nominal.pendulumDamping = 0;
example = nominal;
% The same assumed viscous coefficients as demo_furuta_milestone1.
example.armDamping = 1.1e-4;
example.pendulumDamping = 2.4e-5;

equilibrium = [0;pi;0;0];
undamped = analyzeCase(equilibrium,nominal);
damped = analyzeCase(equilibrium,example);
tol = 1e-8;
u = undamped.poles;
d = damped.poles;
assert(nnz(abs(u)<tol)==2 && rank(undamped.A,tol)==3 && rank(undamped.A^2,tol)==2, ...
    'Expected a double zero pole with one independent eigenvector.');
assert(nnz(abs(imag(u))>tol)==2 && all(abs(real(u))<tol), ...
    'Expected an undamped oscillatory pair.');
assert(nnz(abs(d)<tol)==1 && nnz(real(d)<-tol)==3, ...
    'Expected one neutral arm-angle mode and three decaying modes.');

% Compare all four small-signal states against the nonlinear equations.
cases = ["Undamped";"Example damping"];
models = {undamped,damped};
poleTable = table;
for k = 1:numel(models)
    poles = models{k}.poles;
    poleTable = [poleTable;table(repmat(cases(k),4,1),real(poles),imag(poles), ...
        'VariableNames',{'Case','Real_per_s','Imag_per_s'})]; %#ok<AGROW>
end
folder = fullfile(root,'output','multibody');
results = struct('equilibrium',equilibrium,'undamped',undamped,'damped',damped);
save(fullfile(folder,'hanging_state_space.mat'),'-struct','results');
writetable(poleTable,fullfile(folder,'hanging_equilibrium_poles.csv'));
disp(poleTable);
fprintf('Finite-difference Jacobian error (undamped/damped): %.3g / %.3g\n', ...
    undamped.jacobianError,damped.jacobianError);
fprintf('Nonlinear comparison max angle error rad: %.3g / %.3g\n', ...
    undamped.maxAngleError,damped.maxAngleError);
fprintf('Nonlinear comparison max rate error rad/s: %.3g / %.3g\n', ...
    undamped.maxRateError,damped.maxRateError);
end

function result = analyzeCase(xeq,p)
f = @(x,tau)furuta_cad_dynamics(x,tau,p);
assert(norm(f(xeq,0),inf)<1e-9,'Hanging is not an equilibrium.');
A = zeros(4);
finiteDifference = zeros(4);
for k = 1:4
    direction = zeros(4,1);
    direction(k) = 1;
    A(:,k) = imag(f(xeq+1i*1e-20*direction,0))/1e-20;
    h = 1e-5;
    finiteDifference(:,k) = (f(xeq+h*direction,0)-f(xeq-h*direction,0))/(2*h);
end
B = imag(f(xeq,1i*1e-20))/1e-20;
jacobianError = max(abs(A-finiteDifference),[],'all');
assert(jacobianError<1e-6,'The Jacobian methods disagree.');
assert(norm(B-(f(xeq,1e-4)-f(xeq,-1e-4))/2e-4,inf)<1e-7, ...
    'The input linearizations disagree.');
C = [eye(2) zeros(2)];
D = zeros(2,1);
poles = eig(A);

t = linspace(0,1,201).';
offset = [0;0.005;0.02;0];
[~,x] = ode45(@(~,x)f(x,0),t,xeq+offset,odeset('RelTol',1e-11,'AbsTol',1e-13));
linear = zeros(numel(t),4);
for k = 1:numel(t)
    linear(k,:) = (xeq+expm(A*t(k))*offset).';
end
angleError = max(abs(x(:,1:2)-linear(:,1:2)),[],'all');
rateError = max(abs(x(:,3:4)-linear(:,3:4)),[],'all');
assert(angleError<1e-5 && rateError<1e-4,'Small-signal response does not match nonlinear model.');
result = struct('A',A,'B',B,'C',C,'D',D,'poles',poles, ...
    'armDamping',p.armDamping,'pendulumDamping',p.pendulumDamping, ...
    'jacobianError',jacobianError,'maxAngleError',angleError,'maxRateError',rateError);
end
