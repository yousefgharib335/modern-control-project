function verify_furuta_plant(source)
% Check the analytical linearization and the behavior of the unforced plant.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
p = furuta_parameters(source);
[A,B] = furuta_linearize(p);
[Adown,Bdown] = furuta_linearize(p,pi);
checkJacobian(p,0,A,B);
checkJacobian(p,pi,Adown,Bdown);
assert(max(real(eig(A))) > 0,'Upright must have an unstable mode.');
assert(max(real(eig(Adown))) < 1e-8, ...
    'Hanging equilibrium should have no unstable modes.');

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[~,xUp] = ode45(@(~,x)furuta_dynamics(x,0,p),[0 0.25],[0; 0.01; 0; 0],opts);
assert(abs(xUp(end,2)) > abs(xUp(1,2)),'Upright disturbance did not grow.');
x0 = [0; pi+0.1; 0; 0];
[~,xDown] = ode45(@(~,x)furuta_dynamics(x,0,p),[0 3],x0,opts);
assert(max(abs(xDown(:,2)-pi)) < 0.101, ...
    'Hanging displacement grew beyond its initial value.');
if source == "example"
    assert(min(xDown(:,2)) < pi,'Example hanging response did not oscillate.');
end

model = char("furuta_plant_" + source);
modelFile = fullfile(root,'models',[model '.slx']);
if isfile(modelFile)
    alreadyOpen = bdIsLoaded(model);
    load_system(modelFile);
    if ~alreadyOpen
        closeModel = onCleanup(@()close_system(model,0));
    end
    input = Simulink.SimulationInput(model);
    input = input.setModelParameter('StopTime','3','SaveTime','on', ...
        'SaveOutput','on','SaveFormat','Array','TimeSaveName','tout','OutputSaveName','yout');
    for voltage = [0 0.1]
        drivenInput = input.setExternalInput([0 voltage; 3 voltage]);
        result = sim(drivenInput);
        [~,reference] = ode45(@(~,x)furuta_dynamics(x,voltage,p),result.tout,x0,opts);
        assert(max(abs(result.yout-reference),[],'all') < 1e-5, ...
            'Simulink and MATLAB ODE states differ at %.2f V.',voltage);
    end
    disp('Simulink and MATLAB ODE trajectories agree at 0 and 0.1 V.');
else
    fprintf('Build %s to compare the Simulink trajectory.\n',model);
end
fprintf('Plant checks passed using %s parameters.\n',source);
disp('Upright poles:'); disp(eig(A).');
disp('Hanging poles:'); disp(eig(Adown).');
end

function checkJacobian(p, angle, A, B)
x = [0; angle; 0; 0];
assert(norm(furuta_dynamics(x,0,p),inf) < 1e-8,'Equilibrium residual is too large.');
h = 1e-6;
steps = h*eye(4);
numericA = zeros(4);
for k = 1:4
    numericA(:,k) = (furuta_dynamics(x+steps(:,k),0,p) ...
        - furuta_dynamics(x-steps(:,k),0,p))/(2*h);
end
numericB = (furuta_dynamics(x,h,p)-furuta_dynamics(x,-h,p))/(2*h);
assert(norm(numericA-A,inf) < 1e-6 && norm(numericB-B,inf) < 1e-6, ...
    'Analytical linearization disagrees with finite differences.');
end
