function verify_furuta_voltage_plant
% Compare the complete voltage-driven model with the coupled nonlinear ODE.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
model = 'furuta_multibody_voltage';
if ~bdIsLoaded(model)
    modelPath = fullfile(root,'models',[model '.slx']);
    if ~isfile(modelPath)
        build_furuta_multibody("voltage");
    end
    load_system(modelPath);
    cleanup = onCleanup(@()close_system(model,0));
end
workspace = get_param(model,'ModelWorkspace');
drive = getVariable(workspace,'drive');
p = getVariable(workspace,'p');
initialState = [0; pi; 0; 0];
command = [0 0; 0.15 0; 0.17 3; 0.45 3; 0.47 14; 0.55 14; ...
    0.57 -14; 0.65 -14; 0.67 -3; 0.95 -3; 0.97 0; 1.3 0];

input = Simulink.SimulationInput(model);
input = input.setVariable('initialState',initialState,'Workspace',model);
input = input.setExternalInput(command);
input = input.setModelParameter('StopTime','1.3','SaveTime','on','SaveOutput','on', ...
    'SimMechanicsOpenEditorOnUpdate','off', ...
    'SaveFormat','Array','TimeSaveName','tout','OutputSaveName','yout');
result = sim(input);
t = result.tout;
y = result.yout;
assert(isequal(size(y),[numel(t) 6]) && all(isfinite(y),'all'), ...
    'FurutaMotor:Outputs','Expected four states, current and drive torque.');
x = y(:,1:4);
current = y(:,5);
torque = y(:,6);
voltage = interp1(command(:,1),command(:,2),t);
limited = min(max(voltage,-drive.voltageLimit),drive.voltageLimit);
expectedCurrent = (limited-drive.backEmfOutput*x(:,3))/drive.resistance;
expectedTorque = drive.torqueConstantOutput*expectedCurrent-drive.viscousDamping*x(:,3);
currentError = max(abs(current-expectedCurrent));
torqueError = max(abs(torque-expectedTorque));
assert(currentError<1e-8 && torqueError<1e-9,'FurutaMotor:Drive', ...
    'Current or torque differs from the output-side equations.');
assert(abs(current(1))<1e-8 && abs(torque(1))<1e-8 && ...
    max(torque)>0.1 && min(torque)<-0.1 && max(abs(x(:,1)))>0.1, ...
    'FurutaMotor:Excitation','Expected motion and both signs of torque.');

opts = odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',0.0005);
[~,reference] = ode45(@(time,state) drivenDynamics(time,state,command,drive,p), ...
    t,initialState,opts);
error = x-reference;
error(:,1:2) = atan2(sin(error(:,1:2)),cos(error(:,1:2)));
angleError = max(abs(error(:,1:2)),[],'all');
rateError = max(abs(error(:,3:4)),[],'all');
assert(angleError<1e-5 && rateError<1e-4,'FurutaMotor:MechanicalMatch', ...
    'Simscape and analytical trajectories differ: %.3g rad, %.3g rad/s.',angleError,rateError);
noLoadRPM = drive.voltageLimit*drive.speedGain*60/(2*pi);
assert(abs(noLoadRPM-200)<1,'FurutaMotor:NominalSpeed','Unloaded speed fit differs from 200 RPM.');
fprintf('Voltage plant checks passed. Unloaded speed %.2f RPM.\n',noLoadRPM);
fprintf('Peak errors: %.3g A, %.3g N*m, %.3g rad, %.3g rad/s.\n', ...
    currentError,torqueError,angleError,rateError);
end

function dx = drivenDynamics(t,x,command,drive,p)
voltage = interp1(command(:,1),command(:,2),t);
voltage = min(max(voltage,-drive.voltageLimit),drive.voltageLimit);
current = (voltage-drive.backEmfOutput*x(3))/drive.resistance;
torque = drive.torqueConstantOutput*current-drive.viscousDamping*x(3);
dx = furuta_cad_dynamics(x,torque,p);
end
