function verify_furuta_l298n
% Exercise saturated drive, both directions, disabled coasting and braking.
root = fileparts(mfilename('fullpath'));
model = 'furuta_multibody_l298n';
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
drive = getVariable(get_param(model,'ModelWorkspace'),'drive');
command = [0 0 0; .10 0 0; .12 16 0; .32 16 0; .34 0 0; .46 0 0; ...
    .47 0 1; .57 0 1; .58 0 0; .60 -16 0; .79 -16 0; .81 0 0; ...
    .85 6 0; .95 6 0; .97 0 0; 1.1 0 0];
input = Simulink.SimulationInput(model);
input = input.setVariable('initialState',[0;pi;0;0],'Workspace',model);
input = input.setExternalInput(command);
input = input.setModelParameter('StopTime','1.1','SaveTime','on','TimeSaveName','tout', ...
    'SimMechanicsOpenEditorOnUpdate','off', ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Array');
result = sim(input);
t = result.tout;
y = result.yout;
assert(isequal(size(y),[numel(t) 6]) && all(isfinite(y),'all'), ...
    'FurutaL298N:Output','Expected four plant states, average current and torque.');

controls = interp1(command(:,1),command(:,2:3),t);
voltage = controls(:,1);
braking = controls(:,2)>=0.5;
armSpeed = y(:,3);
backEmf = drive.backEmfOutput*armSpeed;
duty = min(abs(voltage)/drive.voltageLimit,1);
fullDutyVoltage = drive.voltageLimit-drive.bridgeDrop;
expectedCurrent = duty.*(sign(voltage)*fullDutyVoltage-backEmf)/drive.resistance;
expectedCurrent(braking) = -backEmf(braking)/drive.resistance;
expectedTorque = drive.torqueConstantOutput*expectedCurrent-drive.viscousDamping*armSpeed;
assert(max(abs(y(:,5)-expectedCurrent))<1e-8 && max(abs(y(:,6)-expectedTorque))<1e-8, ...
    'FurutaL298N:Drive','Driver or motor outputs differ from the averaged equations.');
coast = t>.35 & t<.45;
brake = t>.48 & t<.56;
assert(any(coast) && any(brake) && max(abs(y(coast,5)))<1e-8 && ...
    min(y(brake,5))<-.05 && all(y(brake,5).*armSpeed(brake)<0) && ...
    max(y(:,5))>0.5 && min(y(:,5))<-.5 && max(abs(armSpeed))>1, ...
    'FurutaL298N:Modes','Expected forward, reverse, coast and brake behavior.');

output = fullfile(root,'output','multibody');
if ~isfolder(output)
    mkdir(output);
end
trace = array2table([t controls y],'VariableNames',{'Time_s','RequestedVoltage_V', ...
    'BrakeCommand','Theta0_rad','Theta1_rad','Omega0_rad_s','Omega1_rad_s','AverageCurrent_A','DriveTorque_Nm'});
writetable(trace,fullfile(output,'l298n_driver_response.csv'));
fprintf('L298N simulation checks passed: forward, reverse, half duty, saturation, coast and brake.\n');
fprintf('Assumed full-duty voltage %.2f V. Peak simulated average current %.3g A.\n', ...
    fullDutyVoltage,max(abs(y(:,5))));
end
