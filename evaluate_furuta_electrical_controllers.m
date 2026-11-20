function evaluate_furuta_electrical_controllers(variants,test)
% Compare recovery from the default pendulum push or the original release.
arguments
    variants (1,:) string {mustBeMember(variants,["pid","pole_placement","lqr"])} = ["pid","pole_placement","lqr"]
    test (1,1) string {mustBeMember(test,["push","release"])} = "push"
end
root = fileparts(mfilename('fullpath'));
folder = fullfile(root,'output','multibody');
passed = false(size(variants));
for k = 1:numel(variants)
    passed(k) = evaluateController(root,folder,variants(k),test);
end
if any(~passed)
    error('FurutaElectricalControllers:Recovery', ...
        'Failed the %s test: %s.',test,strjoin(cellstr(variants(~passed)),', '));
end
compare_furuta_electrical_controllers(test);
end

function passed = evaluateController(root,folder,variant,test)
model = char("furuta_" + variant + "_model");
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
assert(getSimulinkBlockHandle([model '/Small Command Compensation'])~=-1, ...
    'Rebuild %s before running the compensated disturbance test.',model);
stopTime = 5;
input = Simulink.SimulationInput(model);
prefix = "electrical_" + variant + "_compensated_";
disturbanceEnd = 0;
if test=="release"
    input = input.setVariable('initialState',[0;deg2rad(1);0;0],'Workspace',model);
    input = input.setVariable('pushTorque',0,'Workspace',model);
else
    input = input.setVariable('initialState',zeros(4,1),'Workspace',model);
    workspace = get_param(model,'ModelWorkspace');
    pushStart = getVariable(workspace,'pushStart');
    pushTorque = getVariable(workspace,'pushTorque');
    disturbanceEnd = pushStart+getVariable(workspace,'pushDuration');
    prefix = prefix + "push_";
end
input = input.setExternalInput([0 0 0;stopTime 0 0]);
input = input.setModelParameter('StopTime',num2str(stopTime), ...
    'SimMechanicsOpenEditorOnUpdate','off', ...
    'SaveTime','on','TimeSaveName','tout','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Dataset');
result = sim(input);
assert(abs(result.tout(end)-stopTime)<1e-8,'%s did not reach its scheduled end time.',model);
sampleTimes = (0:.001:stopTime).';
y = [sampleSignal(result.yout{1}.Values,sampleTimes,'linear'), ...
    sampleSignal(result.yout{2}.Values,sampleTimes,'linear'), ...
    sampleSignal(result.yout{6}.Values,sampleTimes,'previous')];
assert(all(isfinite(y),'all'),'%s produced invalid outputs.',model);
trace = array2table([sampleTimes rad2deg(y(:,1:4)) y(:,5:6)], ...
    'VariableNames',{'Time_s','Arm_deg','Pendulum_deg','ArmRate_deg_s','PendulumRate_deg_s', ...
    'EquivalentRequest_V','BridgeRequest_V'});
if test=="push"
    trace.PushTorque_Nm = sampleSignal(result.yout{7}.Values,sampleTimes,'previous');
    beforePush = sampleTimes<pushStart-.001;
    duringPush = sampleTimes>pushStart+.001 & sampleTimes<disturbanceEnd-.001;
    afterPush = sampleTimes>disturbanceEnd+.001;
    assert(max(abs(y(beforePush,1:4)),[],'all')<1e-7 && ...
        any(duringPush) && all(abs(trace.PushTorque_Nm(duringPush)-pushTorque)<1e-10) && ...
        all(abs(trace.PushTorque_Nm(beforePush | afterPush))<1e-10) && ...
        max(abs(y(afterPush,2)))>deg2rad(0.1), ...
        'FurutaElectricalControllers:Push','%s did not execute the expected upright push.',model);
    traceFile = "response.csv";
else
    traceFile = "release.csv";
end
writetable(trace,fullfile(folder,prefix + traceFile));

% Recovery requires both angles and rates to stay inside these tolerances.
limits = [0.5 0.1 1 1];
states = abs(rad2deg(y(:,1:4)));
outside = find(sampleTimes>=disturbanceEnd & any(states>limits,2),1,'last');
settlingTime = NaN;
if isempty(outside)
    settlingTime = 0;
elseif outside<numel(sampleTimes)
    settlingTime = sampleTimes(outside+1)-disturbanceEnd;
end
tail = max(states(sampleTimes>=stopTime-1,:),[],1);
summary = array2table([settlingTime,tail,max(abs(y(:,6))),trace.Pendulum_deg(end),trace.Arm_deg(end)], ...
    'VariableNames',{'Settling_s','LastSecondMaxArm_deg','LastSecondMaxPendulum_deg', ...
    'LastSecondMaxArmRate_deg_s','LastSecondMaxPendulumRate_deg_s','PeakBridgeRequest_V', ...
    'FinalPendulum_deg','FinalArm_deg'});
if test=="push"
    summary.PeakPendulum_deg = max(abs(trace.Pendulum_deg));
end
writetable(summary,fullfile(folder,prefix + "metrics.csv"));
fprintf('%s %s test:\n',model,test);
disp(summary);
% Apply the same four-second recovery budget from the end of the disturbance.
passed = max(abs(trace.Pendulum_deg))<=2 && settlingTime<=4;
end

function sampled = sampleSignal(signal,sampleTimes,method)
count = numel(signal.Time);
if signal.IsTimeFirst
    data = reshape(signal.Data,count,[]);
else
    data = reshape(signal.Data,[],count).';
end
[time,index] = unique(signal.Time,'last');
sampled = interp1(time,data(index,:),sampleTimes,method,'extrap');
end
