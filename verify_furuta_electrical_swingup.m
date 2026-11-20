function result = verify_furuta_electrical_swingup
% Verify that the existing swing-up law actually drives the switching bridge.
root = fileparts(mfilename('fullpath'));
model = 'furuta_multibody_electrical_swingup';
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
cfg = getVariable(get_param(model,'ModelWorkspace'),'cfg');
stopTime = 0.5;
input = Simulink.SimulationInput(model);
input = input.setBlockParameter([model '/Drive Command'],'LabelModeActiveChoice','SwingUp');
input = input.setExternalInput([0 0 0;stopTime 0 0]);
input = input.setModelParameter('StopTime',num2str(stopTime), ...
    'SimMechanicsOpenEditorOnUpdate','off','SaveTime','on','TimeSaveName','tout', ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Array');
result = sim(input);
y = result.yout;
t = result.tout;
assert(size(y,2)==7 && all(isfinite(y),'all') && abs(t(end)-stopTime)<1e-8, ...
    'FurutaElectricalSwingUp:Simulation','Missing or incomplete electrical swing-up output.');
assert(norm(y(1,1:4).'-[0;pi;0;0],inf)<1e-7,'Unexpected initial pendulum state.');
assert(max(abs(y(:,5)))>1e-3 && max(abs(y(:,6)))>1e-4 && ...
    max(abs(y(:,1)))>deg2rad(0.1),'Swing-Up did not actuate the electrical plant.');
uprightError = abs(atan2(sin(y(:,2)),cos(y(:,2))));
capture = uprightError<cfg.captureAngle & abs(y(:,4))<cfg.captureRate & ...
    abs(y(:,3))<cfg.captureArmRate;
fprintf('Electrical Swing-Up wiring passed with both external commands at zero.\n');
fprintf('In %.1f s, the closest approach was %.1f deg from upright.\n', ...
    stopTime,rad2deg(min(uprightError)));
fprintf('Samples inside the provisional angle and speed capture limits: %d.\n',nnz(capture));
end
