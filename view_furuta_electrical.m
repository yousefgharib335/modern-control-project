function simulation = view_furuta_electrical(mode,stopTime)
% Play the manual drive demo or the automatic swing-up test in the 3D viewer.
arguments
    mode (1,1) string {mustBeMember(mode,["manual","swingup"])} = "manual"
    stopTime (1,1) double {mustBeFinite,mustBePositive} = 1.35
end
root = fileparts(mfilename('fullpath'));
if mode=="swingup"
    model = 'furuta_multibody_electrical_swingup';
    command = [0 0 0;stopTime 0 0];
    driveChoice = 'SwingUp';
else
    model = 'furuta_multibody_electrical';
    command = [0 0 0; .10 0 0; .11 4 0; .37 4 0; .38 0 0; ...
        .48 0 0; .49 0 1; .63 0 1; .64 0 0; .70 0 0; ...
        .71 -4 0; .97 -4 0; .98 0 0; max(stopTime,1.35) 0 0];
    driveChoice = 'Manual';
end
open_system(fullfile(root,'models',[model '.slx']));
input = Simulink.SimulationInput(model);
if mode=="swingup"
    input = input.setBlockParameter([model '/Drive Command'],'LabelModeActiveChoice',driveChoice);
end
input = input.setVariable('initialState',[0;pi;0;0],'Workspace',model);
input = input.setExternalInput(command);
input = input.setModelParameter('StopTime',num2str(stopTime), ...
    'SimMechanicsOpenEditorOnUpdate','on','SaveTime','on','TimeSaveName','tout', ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Array');
simulation = sim(input);
fprintf('Electrical plant demo complete (%s). Use Play in Multibody Explorer.\n',driveChoice);
end
