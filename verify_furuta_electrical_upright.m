function verify_furuta_electrical_upright
% Measure cycle-average winding current around upright with brake released.
root = fileparts(mfilename('fullpath'));
model = 'furuta_multibody_electrical';
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
drive = getVariable(get_param(model,'ModelWorkspace'),'drive');
period = 1/drive.pwmFrequency;
levels = [0 .03 -.03 .15 -.15 .35 -.35 .7 -.7 1.4 -1.4 2.8 -2.8 ...
    .25 -.25 .5 -.5 1 -1 2 -2];
duration = 8*period;
edges = (0:numel(levels)).'*duration;
start = edges(1:end-1);
finish = edges(2:end)-period/100;
time = reshape([start finish].',[],1);
requested = repelem(levels(:),2);
input = Simulink.SimulationInput(model);
input = input.setVariable('initialState',zeros(4,1),'Workspace',model);
input = input.setExternalInput([time requested zeros(size(time));edges(end) levels(end) 0]);
input = input.setModelParameter('StopTime',num2str(edges(end)), ...
    'SimMechanicsOpenEditorOnUpdate','off', ...
    'SaveTime','on','TimeSaveName','tout','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Array');
result = sim(input);
t = result.tout;
y = result.yout;
assert(size(y,2)==7 && all(isfinite(y),'all'),'Invalid electrical plant outputs.');
[t,index] = unique(t,'last');
y = y(index,:);
average = zeros(numel(levels),3);
for k = 1:numel(levels)
    a = start(k)+2*period;
    b = start(k)+7*period;
    window = [a; t(t>a & t<b); b];
    average(k,:) = trapz(window,interp1(t,y(:,[3 5 6]),window))/(b-a);
end
voltage = levels(:);
duty = abs(voltage)/drive.voltageLimit;
predicted = duty.*(sign(voltage)*(drive.voltageLimit-drive.bridgeDrop) ...
    -drive.backEmfOutput*average(:,1))/drive.resistance;
comparison = table(voltage,average(:,1),average(:,2),predicted,average(:,3), ...
    'VariableNames',{'Request_V','MeanArmRate_rad_s','SwitchingCurrent_A', ...
    'ExistingAverageCurrent_A','SwitchingTorque_Nm'});
validation = abs(voltage)==.25 | abs(voltage)==.5 | abs(voltage)==1 | abs(voltage)==2;
positive = ~validation & voltage>0;
negative = ~validation & voltage<0;
voltageKnots = [0;voltage(positive)];
currentKnots = [0;(average(positive,2)-average(negative,2))/2];
assert(all(abs(average(abs(voltage)<=.03,2))<1e-7), ...
    'The assumed zero-response band no longer matches the switching model.');
currentKnots(voltageKnots<=.03) = 0;
comparison.FittedAverageCurrent_A = sign(voltage).*interp1(voltageKnots,currentKnots,abs(voltage),'pchip');
comparison.HeldOut = validation;
oldError = sqrt(mean((comparison.ExistingAverageCurrent_A(validation)-average(validation,2)).^2));
newError = sqrt(mean((comparison.FittedAverageCurrent_A(validation)-average(validation,2)).^2));
fprintf('Held-out current RMSE, previous average %.4g A, fitted average %.4g A.\n',oldError,newError);
assert(newError<oldError,'FurutaElectrical:AverageFit','The fit did not improve held-out current agreement.');
writetable(comparison,fullfile(root,'output','multibody','electrical_upright_pwm_comparison.csv'));
folder = fullfile(root,'output','multibody');
save(fullfile(folder,'electrical_upright_average_fit.mat'),'voltageKnots','currentKnots','oldError','newError','drive');
end
