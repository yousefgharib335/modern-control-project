function add_furuta_pid_block(controller,position,g)
% Build a PID/PD subsystem with a continuous controller state.
folder = fileparts(mfilename('fullpath'));
add_block('simulink/Ports & Subsystems/Subsystem',controller,'Position',position);
Simulink.SubSystem.deleteContents(controller);
makeBlocks(controller,{
    'Arm Angle','Sources/In1',[35 65],{}, {'Port','1'}
    'Pendulum Angle','Sources/In1',[35 155],{}, {'Port','2'}
    'Controller State','Continuous/Integrator',[510 240],{}, {'InitialCondition','[0;initialState(1:2)]'}
    'Voltage','Sinks/Out1',[660 65],{}, {'Port','1'}
    'Unsaturated Voltage','Sinks/Out1',[660 155],{}, {'Port','2'}
});
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [controller '/PID Function'],'Position',[225 65 405 175]);
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',[controller '/PID Function']);
assignments = strings(0,1);
for name = string(fieldnames(g)).'
    assignments(end+1,1) = sprintf('g.%s = %s;',name,mat2str(g.(name),17));
end
chart.Script = char(strjoin([ ...
    "function [voltage, requestedVoltage, stateDerivative] = controller(theta0,theta1,state)"; ...
    assignments; ...
    "[voltage, requestedVoltage, stateDerivative] = furuta_pid(theta0,theta1,state,g);"; ...
    "end"; string(fileread(fullfile(folder,'furuta_pid.m')))],newline));
wire(controller,'Arm Angle','PID Function/1');
wire(controller,'Pendulum Angle','PID Function/2');
wire(controller,'Controller State','PID Function/3');
wire(controller,'PID Function/1','Voltage/1');
wire(controller,'PID Function/2','Unsaturated Voltage/1');
wire(controller,'PID Function/3','Controller State/1');
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
