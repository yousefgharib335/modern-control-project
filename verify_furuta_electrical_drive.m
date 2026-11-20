function verify_furuta_electrical_drive
% Exercise the Simscape switching bridge against the CAD mechanical load.
root = fileparts(mfilename('fullpath'));
model = 'furuta_multibody_electrical';
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
drive = getVariable(get_param(model,'ModelWorkspace'),'drive');
command = [0 0 0; .005 0 0; .006 6 0; .045 6 0; .046 0 0; ...
    .052 0 0; .053 0 1; .067 0 1; .068 0 0; .075 0 0; ...
    .076 -6 0; .095 -6 0; .096 0 0; .105 0 0];
input = Simulink.SimulationInput(model);
input = input.setVariable('initialState',[0;pi;0;0],'Workspace',model);
input = input.setExternalInput(command);
input = input.setModelParameter('StopTime','0.105','SaveTime','on','TimeSaveName','tout', ...
    'SimMechanicsOpenEditorOnUpdate','off', ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Array');
result = sim(input);
t = result.tout;
y = result.yout;
assert(isequal(size(y),[numel(t) 7]) && all(isfinite(y),'all'), ...
    'FurutaElectrical:Output','Expected four states, winding current, torque, and terminal voltage.');
speed = y(:,3);
current = y(:,5);
torque = y(:,6);
terminalVoltage = y(:,7);
forward = t>.012 & t<.043;
reverse = t>.08 & t<.093;
coast = t>.048 & t<.051;
brake = t>.056 & t<.065;
assert(max(current(forward))>.5 && min(current(reverse))<-.5 && max(speed(forward))>.1, ...
    'FurutaElectrical:Direction','Expected bidirectional current and arm motion.');
assert(max(terminalVoltage(forward))-min(terminalVoltage(forward))>6 && ...
    max(current(forward))-min(current(forward))>.4, ...
    'FurutaElectrical:Switching','Expected visible PWM voltage and current ripple.');
coastCurrent = max(abs(current(coast)));
brakeCurrent = min(current(brake));
brakePowerSign = max(current(brake).*speed(brake));
assert(coastCurrent<.01 && brakeCurrent<-.02 && brakePowerSign<0, ...
    'FurutaElectrical:Brake','Coast %.4g A, minimum brake %.4g A, max brake current times speed %.4g.', ...
    coastCurrent,brakeCurrent,brakePowerSign);
expectedTorque = drive.torqueConstantOutput*current-drive.viscousDamping*speed;
assert(max(abs(expectedTorque-torque))<1e-7, ...
    'FurutaElectrical:Torque','Current-to-shaft torque coupling differs.');

% Integrated winding law during the PWM interval, using measured terminal voltage.
dt = t(forward);
di = current(forward);
resistiveVoltage = drive.resistance*di;
backEmf = drive.backEmfOutput*speed(forward);
deltaFlux = drive.armatureInductance*(di-di(1));
integratedVoltage = cumtrapz(dt,terminalVoltage(forward)-resistiveVoltage-backEmf);
assert(max(abs(deltaFlux-integratedVoltage))<2e-4, ...
    'FurutaElectrical:Winding','Integrated R-L and back-EMF voltage balance differs.');

output = fullfile(root,'output','multibody');
if ~isfolder(output)
    mkdir(output);
end
trace = array2table([t y],'VariableNames',{'Time_s','Theta0_rad','Theta1_rad', ...
    'Omega0_rad_s','Omega1_rad_s','WindingCurrent_A','DriveTorque_Nm','TerminalVoltage_V'});
writetable(trace,fullfile(output,'electrical_drive_response.csv'));
fprintf('PWM circuit checks passed: switching, forward, reverse, coast, brake and torque coupling.\n');
fprintf('Provisional %g mH, %g Hz. Peak winding current %.3g A for this test.\n', ...
    1e3*drive.armatureInductance,drive.pwmFrequency,max(abs(current)));
end
