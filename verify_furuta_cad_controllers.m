function verify_furuta_cad_controllers
% Exercise each controller's connection to the switching drive without tuning.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'models'));
models = {'furuta_pid_model','furuta_pole_placement_model','furuta_lqr_model','furuta_hybrid_model'};
for k = 1:numel(models)
    checkModel(models{k},k==numel(models));
end
end

function checkModel(model,isHybrid)
testVoltage = 0.05;
stopTime = 0.035;
if ~bdIsLoaded(model)
    load_system(model);
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
workspace = get_param(model,'ModelWorkspace');
initialState = getVariable(workspace,'initialState');
drive = getVariable(workspace,'drive');
input = Simulink.SimulationInput(model);
if ~isHybrid
    % Excite the controller before the default push time in this short test.
    initialState = [0;deg2rad(1);0;0];
    input = input.setVariable('initialState',initialState,'Workspace',model);
end
command = [0 testVoltage 0; .010 testVoltage 0; .011 testVoltage 1; ...
    .016 testVoltage 1; .017 testVoltage 0; stopTime testVoltage 0];
input = input.setExternalInput(command);
input = input.setModelParameter('StopTime',num2str(stopTime), ...
    'SimMechanicsOpenEditorOnUpdate','off', ...
    'SaveOutput','on','SaveFormat','Dataset','OutputSaveName','yout');
voltageSource = 'Requested Voltage V';
if isHybrid
    voltageSource = 'Drive Enabled';
    input = input.setVariable('disableTime',0.02,'Workspace',model);
end
ports = get_param([model '/System'],'PortHandles');
assert(strcmp(sourceName(ports.Inport(1)),voltageSource) && ...
    strcmp(sourceName(ports.Inport(2)),'Brake Command'), ...
    'FurutaControllers:Wiring','Incorrect System inputs in %s.',model);

result = sim(input);
signals = result.yout;
state = signals{1}.Values;
assert(all(isfinite(state.Data),'all') && norm(sampleAt(state,0)-initialState,inf)<1e-7, ...
    'FurutaControllers:State','Invalid state or initial condition in %s.',model);
assert(abs(sampleAt(signals{2}.Values,0.008))>1e-4, ...
    'FurutaControllers:Inactive','Missing controller excitation in %s.',model);

current = signals{4}.Values;
voltage = signals{5+isHybrid}.Values;
active = current.Time>0.003 & current.Time<0.01;
pwm = voltage.Time>0.003 & voltage.Time<0.01;
terminalVoltage = voltage.Data(pwm);
assert(max(abs(current.Data(active)))>1e-3 && max(terminalVoltage)-min(terminalVoltage)>4, ...
    'FurutaControllers:Switching','Missing switched voltage or motor current in %s.',model);
for time = [0.008 0.014]
    speed = sampleAt(state,time);
    expectedTorque = drive.torqueConstantOutput*sampleAt(current,time)-drive.viscousDamping*speed(3);
    assert(abs(sampleAt(signals{3}.Values,time)-expectedTorque)<3e-5, ...
        'FurutaControllers:Torque','Unexpected current-to-torque coupling in %s.',model);
end
if isHybrid
    mode = signals{5}.Values;
    assert(sampleAt(mode,0.008)==1 && sampleAt(mode,0.032)==0, ...
        'FurutaControllers:Supervisor','Expected Swing-Up and Safe-Stop modes.');
    assert(abs(sampleAt(signals{2}.Values,0.032))<1e-10 && abs(sampleAt(current,0.032))<1e-3, ...
        'FurutaControllers:Stop','Safe-Stop still drives the motor.');
end
fprintf('%s passed electrical command, state feedback, and torque checks.\n',model);
end

function name = sourceName(inputPort)
line = get_param(inputPort,'Line');
source = get_param(line,'SrcBlockHandle');
name = get_param(source,'Name');
end

function value = sampleAt(series,time)
sample = resample(series,time);
value = sample.Data(:);
end
