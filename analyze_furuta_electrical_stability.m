function results = analyze_furuta_electrical_stability
% Separate the zero-command Jacobian from the finite-amplitude design model.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
folder = fullfile(root,'output','multibody');
fit = load(fullfile(folder,'electrical_upright_average_fit.mat'));
model = 'furuta_multibody_electrical';
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
workspace = get_param(model,'ModelWorkspace');
p = getVariable(workspace,'p');
drive = getVariable(workspace,'drive');
assert(all(fit.currentKnots(fit.voltageKnots<=.03)==0), ...
    'Expected the measured averaged map to be flat around zero.');

upright = linearize([0;0;0;0],p,drive);
hanging = linearize([0;pi;0;0],p,drive);
assert(nnz(real(upright.poles)>1e-8)==1 && ...
    nnz(abs(hanging.poles)<1e-8)==1 && nnz(real(hanging.poles)<-1e-8)==3, ...
    'Unexpected zero-command equilibrium poles.');
assert(upright.observability==4 && hanging.observability==4, ...
    'The angle measurements should observe all four mechanical states.');

% This secant model is a tuning heuristic only. It is not the Jacobian.
referenceVoltage = 0.7;
referenceCurrent = interp1(fit.voltageKnots,fit.currentKnots,referenceVoltage,'pchip');
designB = upright.torqueB*(drive.torqueConstantOutput*referenceCurrent/referenceVoltage);
designRank = controllabilityRank(upright.A,designB);
assert(designRank==4,'The finite-amplitude design model should be reachable.');
results = struct('upright',upright,'hanging',hanging,'referenceVoltage',referenceVoltage, ...
    'designB',designB,'designRank',designRank);
save(fullfile(folder,'electrical_equilibrium_state_space.mat'),'-struct','results');
poleTable = table(repelem(["Upright";"Hanging"],4), ...
    real([upright.poles;hanging.poles]),imag([upright.poles;hanging.poles]), ...
    repelem([upright.controllability;hanging.controllability],4), ...
    repelem([upright.observability;hanging.observability],4), ...
    'VariableNames',{'Equilibrium','Real_per_s','Imag_per_s','Controllability','Observability'});
writetable(poleTable,fullfile(folder,'electrical_equilibrium_poles.csv'));
disp(poleTable);
end

function analysis = linearize(xeq,p,drive)
f = @(x)furuta_cad_dynamics(x,-drive.viscousDamping*x(3),p);
assert(norm(f(xeq),inf)<1e-8,'The proposed operating point is not an equilibrium.');
A = zeros(4);
for k = 1:4
    perturbation = zeros(4,1);
    perturbation(k) = 1i*1e-20;
    A(:,k) = imag(f(xeq+perturbation))/1e-20;
end
torqueB = furuta_cad_dynamics(xeq,1,p)-f(xeq);
B = zeros(4,1); % Zero slope of fitted mean-current map inside the no-response band.
C = [eye(2) zeros(2)];
analysis = struct('xeq',xeq,'A',A,'B',B,'C',C,'D',zeros(2,1),'torqueB',torqueB, ...
    'poles',eig(A),'controllability',controllabilityRank(A,B), ...
    'observability',rank([C;C*A;C*A^2;C*A^3]));
end

function value = controllabilityRank(A,B)
value = rank([B A*B A^2*B A^3*B]);
end
