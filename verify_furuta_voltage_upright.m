function verify_furuta_voltage_upright
% Verify the voltage-input upright linearization against Simscape Multibody.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
model = 'furuta_multibody_voltage';
if ~bdIsLoaded(model)
    modelPath = fullfile(root,'models',[model '.slx']);
    if ~isfile(modelPath)
        build_furuta_multibody("voltage");
    end
    load_system(modelPath);
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
workspace = get_param(model,'ModelWorkspace');
p = getVariable(workspace,'p');
drive = getVariable(workspace,'drive');
[A,B,C,D] = furuta_voltage_linearize(p,drive);
poles = eig(A);
assert(sum(real(poles)>1e-8)==1,'FurutaVoltage:UprightPole','Expected one unstable upright pole.');
assert(rank([B A*B A^2*B A^3*B])==4,'FurutaVoltage:Controllability','Expected four controllable states.');

stopTime = 0.2;
cases = ["equilibrium";"initial_angle";"voltage_step"];
initials = [zeros(4,1) [0;deg2rad(0.1);0;0] zeros(4,1)];
voltages = [0 0 0.02];
angleErrors = zeros(size(cases));
rateErrors = zeros(size(cases));
output = fullfile(root,'output','multibody');
if ~isfolder(output)
    mkdir(output);
end
for k = 1:numel(cases)
    x0 = initials(:,k);
    voltage = voltages(k);
    input = Simulink.SimulationInput(model);
    input = input.setVariable('initialState',x0,'Workspace',model);
    input = input.setExternalInput([0 voltage;stopTime voltage]);
    input = input.setModelParameter('StopTime',num2str(stopTime), ...
        'SimMechanicsOpenEditorOnUpdate','off', ...
        'SaveTime','on','TimeSaveName','tout','SaveOutput','on', ...
        'OutputSaveName','yout','SaveFormat','Array');
    result = sim(input);
    t = result.tout;
    sensed = result.yout(:,1:4);
    assert(all(isfinite(sensed),'all') && norm(sensed(1,:).'-x0,inf)<1e-7, ...
        'FurutaVoltage:InitialState','Initial state or sensed output differs.');

    linear = zeros(size(sensed));
    constantInput = [A B*voltage; zeros(1,5)];
    for j = 1:numel(t)
        augmented = expm(constantInput*t(j))*[x0;1];
        linear(j,:) = (C*augmented(1:4)+D*voltage).';
    end
    difference = sensed-linear;
    angleErrors(k) = max(abs(difference(:,1:2)),[],'all');
    rateErrors(k) = max(abs(difference(:,3:4)),[],'all');
    assert(angleErrors(k)<3e-6 && rateErrors(k)<4e-5, ...
        'FurutaVoltage:LinearMatch','Linear response differs for %s.',cases(k));
    comparison = array2table([t repmat(voltage,numel(t),1) sensed linear], ...
        'VariableNames',{'Time_s','Voltage_V','Theta0_rad','Theta1_rad','Omega0_rad_s','Omega1_rad_s', ...
        'LinearTheta0_rad','LinearTheta1_rad','LinearOmega0_rad_s','LinearOmega1_rad_s'});
    writetable(comparison,fullfile(output,"voltage_upright_"+cases(k)+".csv"));
end
save(fullfile(output,'voltage_upright_state_space.mat'),'A','B','C','D','p','drive','poles');
metrics = table(cases,angleErrors,rateErrors,'VariableNames', ...
    {'Test','MaxAngleError_rad','MaxRateError_rad_s'});
writetable(metrics,fullfile(output,'voltage_upright_metrics.csv'));
disp(metrics);
fprintf('Voltage upright poles (1/s):\n');
disp(poles);
end
