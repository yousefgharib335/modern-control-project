function build_furuta_pid(source)
% Build upright stabilization using a MATLAB Function controller.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
p = furuta_parameters(source);
g = furuta_pid_gains(p);
model = char("furuta_pid_" + source);
plant = char("furuta_plant_" + source);
if bdIsLoaded(model)
    error('FurutaPID:ModelOpen','Close %s before rebuilding it.',model);
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
assignin(w,'g',g);
assignin(w,'initialState',g.initialState);
assignin(w,'disturbanceVoltage',1);
add_block([plant '/Dynamics'],[model '/Plant Dynamics'],'Position',[1040 95 1190 190]);
add_block('simulink/Continuous/Integrator',[model '/Plant State'], ...
    'InitialCondition','initialState','Position',[1250 105 1290 145]);
add_block('simulink/Signal Routing/Demux',[model '/Split State'], ...
    'Outputs','4','Position',[1360 80 1370 240]);

add_furuta_pid_block([model '/PID Controller'],[285 365 495 475],g);

makeBlocks(model,{
    'Disturbance Start','Sources/Step',[540 55],{}, {'Time','4','Before','0','After','disturbanceVoltage'}
    'Disturbance End','Sources/Step',[540 130],{}, {'Time','4.1','Before','0','After','-disturbanceVoltage'}
    'Requested Motor Voltage','Math Operations/Sum',[720 165],{'PID Controller/1','Disturbance Start','Disturbance End'}, {'Inputs','+++'}
    'Driver Limit','Discontinuities/Saturation',[875 160],{'Requested Motor Voltage'}, {'UpperLimit','g.voltageLimit','LowerLimit','-g.voltageLimit'}
    'Results','Signal Routing/Mux',[1510 120],{'Plant State','Driver Limit','PID Controller/2'}, {'Inputs','3'}
    'States and Voltages','Sinks/Out1',[1640 130],{'Results'}, {'Port','1'}
    'Angles','Signal Routing/Mux',[1510 320],{'Split State/1','Split State/2'}, {'Inputs','2'}
    'Angle Scope','Sinks/Scope',[1640 330],{'Angles'}, {}
    'Voltage Scope','Sinks/Scope',[1040 350],{'Driver Limit'}, {}
    'Arm Rate Unused','Sinks/Terminator',[1470 420],{'Split State/3'}, {}
    'Pendulum Rate Unused','Sinks/Terminator',[1470 490],{'Split State/4'}, {}
});
wire(model,'Driver Limit','Plant Dynamics/2');
wire(model,'Plant Dynamics','Plant State/1');
wire(model,'Plant State','Plant Dynamics/1');
wire(model,'Plant State','Split State/1');
wire(model,'Split State/1','PID Controller/1');
wire(model,'Split State/2','PID Controller/2');

set_param(model,'SimulationCommand','update');
save_system(model,fullfile(root,'models',[model '.slx']));
fprintf('Saved PID model using %s parameters.\n',source);
end

function makeBlocks(system,rows)
for k = 1:size(rows,1)
    name = rows{k,1};
    position = rows{k,3};
    parameters = rows{k,5};
    add_block(['simulink/' rows{k,2}],[system '/' name], ...
        'Position',[position position+[65 40]],parameters{:});
end
for k = 1:size(rows,1)
    inputs = rows{k,4};
    for j = 1:numel(inputs)
        wire(system,inputs{j},sprintf('%s/%d',rows{k,1},j));
    end
end
end

function wire(system,from,to)
if ~contains(from,'/')
    from = [from '/1'];
end
add_line(system,from,to,'autorouting','on');
end
