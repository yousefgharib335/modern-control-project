function build_furuta_pole_placement(source)
% Build a nonlinear plant with a full-state pole-placement controller.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
design = furuta_pole_placement_design(furuta_parameters(source));
model = char("furuta_pole_placement_" + source);
plant = char("furuta_plant_" + source);
if bdIsLoaded(model)
    error('FurutaPolePlacement:ModelOpen','Close %s before rebuilding.',model);
end
build_furuta_plant(source);
load_system(fullfile(root,'models',[plant '.slx']));
closePlant = onCleanup(@()close_system(plant,0));
new_system(model);
closeModel = onCleanup(@()close_system(model,0));
set_param(model,'Solver','ode45','RelTol','1e-8','AbsTol','1e-10', ...
    'MaxStep','0.01','StopTime','12');
w = get_param(model,'ModelWorkspace');
plantWorkspace = get_param(plant,'ModelWorkspace');
variables = whos(plantWorkspace);
for k = 1:numel(variables)
    assignin(w,variables(k).name,getVariable(plantWorkspace,variables(k).name));
end
assignin(w,'design',design);
assignin(w,'initialState',design.initialState);
assignin(w,'disturbanceVoltage',1);
add_block([plant '/Dynamics'],[model '/Plant Dynamics'],'Position',[750 115 900 210]);
add_furuta_state_feedback_block([model '/Pole Placement Controller'], ...
    [100 270 300 355],'furuta_pole_placement',design);
rows = {
    'Plant State','Continuous/Integrator',[990 130],{'Plant Dynamics'}, {'InitialCondition','initialState'}
    'Disturbance Start','Sources/Step',[110 35],{}, {'Time','4','Before','0','After','disturbanceVoltage'}
    'Disturbance End','Sources/Step',[110 110],{}, {'Time','4.1','Before','0','After','-disturbanceVoltage'}
    'Motor Command','Math Operations/Sum',[400 145],{'Pole Placement Controller/1','Disturbance Start','Disturbance End'}, {'Inputs','+++'}
    'Driver Limit','Discontinuities/Saturation',[565 140],{'Motor Command'}, {'UpperLimit','design.voltageLimit','LowerLimit','-design.voltageLimit'}
    'Results','Signal Routing/Mux',[1180 130],{'Plant State','Driver Limit','Pole Placement Controller/2'}, {'Inputs','3'}
    'States and Voltages','Sinks/Out1',[1330 140],{'Results'}, {'Port','1'}
    'State Scope','Sinks/Scope',[1180 315],{'Plant State'}, {}
    'Voltage Scope','Sinks/Scope',[750 340],{'Driver Limit'}, {}
};
for k = 1:size(rows,1)
    xy = rows{k,3};
    parameters = rows{k,5};
    add_block(['simulink/' rows{k,2}],[model '/' rows{k,1}], ...
        'Position',[xy xy+[65 40]],parameters{:});
end
for k = 1:size(rows,1)
    inputs = rows{k,4};
    for j = 1:numel(inputs)
        wire(model,inputs{j},sprintf('%s/%d',rows{k,1},j));
    end
end
wire(model,'Driver Limit','Plant Dynamics/2');
wire(model,'Plant State','Plant Dynamics/1');
wire(model,'Plant State','Pole Placement Controller/1');
set_param(model,'SimulationCommand','update');
save_system(model,fullfile(root,'models',[model '.slx']));
fprintf('Saved pole-placement model using %s parameters.\n',source);
end

function wire(system,from,to)
if ~contains(from,'/')
    from = [from '/1'];
end
add_line(system,from,to,'autorouting','on');
end
