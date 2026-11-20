function compare_furuta_electrical_controllers(test)
% Combine the saved disturbance metrics with the poles stored in each model.
arguments
    test (1,1) string {mustBeMember(test,["push","release"])} = "release"
end
root = fileparts(mfilename('fullpath'));
folder = fullfile(root,'output','multibody');
variants = ["pid";"pole_placement";"lqr"];
prefix = "electrical_" + variants + "_compensated_";
filename = "electrical_controller_comparison.csv";
if test=="push"
    prefix = prefix + "push_";
    filename = "electrical_controller_push_comparison.csv";
end
metrics = fullfile(folder,prefix + "metrics.csv");
if ~all(isfile(metrics))
    return;
end
values = zeros(3,4);
poles = strings(3,1);
for k = 1:numel(variants)
    result = readtable(metrics(k));
    values(k,:) = [result.Settling_s,result.PeakBridgeRequest_V,result.FinalPendulum_deg,result.FinalArm_deg];
    poles(k) = modelPoles(root,variants(k));
end
assert(all(values(:,1)<=4),'Run passing disturbance tests before producing the controller comparison.');
comparison = array2table(values,'VariableNames',{'Settling_s','PeakBridgeRequest_V', ...
    'FinalPendulum_deg','FinalArm_deg'});
comparison = addvars(comparison,["PID";"Pole placement";"LQR"], ...
    'Before',1,'NewVariableNames','Controller');
comparison.AverageClosedLoopPoles_per_s = poles;
writetable(comparison,fullfile(folder,filename));
disp(comparison);
end

function formatted = modelPoles(root,variant)
model = char("furuta_" + variant + "_model");
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
workspace = get_param(model,'ModelWorkspace');
if variant=="pid"
    poles = getVariable(workspace,'pidAveragePoles');
else
    design = getVariable(workspace,'stateFeedbackDesign');
    poles = design.actualPoles;
end
formatted = string(mat2str(sort(poles).',5));
end
