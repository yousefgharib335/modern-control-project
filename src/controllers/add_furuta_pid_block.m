function add_furuta_pid_block(controller,position,g)
% Build the arm PID and pendulum PD using continuous Simulink blocks.
add_block('built-in/Subsystem',controller,'Position',position);
addRateFilter([controller '/Arm Rate Filter'],[145 105 245 149],g.filterRate,'initialState(1)');
addRateFilter([controller '/Pendulum Rate Filter'],[145 335 245 379],g.filterRate,'initialState(2)');
makeBlocks(controller,{
    'Arm Angle','Sources/In1',[30 55],{}, {'Port','1'}
    'Pendulum Angle','Sources/In1',[30 290],{}, {'Port','2'}
    'Arm P','Math Operations/Gain',[300 45],{'Arm Angle'}, {'Gain',num2str(g.armP,17)}
    'Arm D','Math Operations/Gain',[300 105],{'Arm Rate Filter'}, {'Gain',num2str(g.armD,17)}
    'Arm I','Math Operations/Gain',[195 215],{'Arm Angle'}, {'Gain',num2str(g.armI,17)}
    'Integral Input','Math Operations/Sum',[300 215],{'Arm I','Anti-Windup Gain'}, {'Inputs','++'}
    'Integral Voltage','Continuous/Integrator',[405 215],{'Integral Input'}, {'InitialCondition','0'}
    'Pendulum P','Math Operations/Gain',[300 280],{'Pendulum Angle'}, {'Gain',num2str(g.pendulumP,17)}
    'Pendulum D','Math Operations/Gain',[300 335],{'Pendulum Rate Filter'}, {'Gain',num2str(g.pendulumD,17)}
    'Requested Voltage','Math Operations/Sum',[535 155], ...
        {'Arm P','Integral Voltage','Arm D','Pendulum P','Pendulum D'}, {'Inputs','+++++'}
    'Voltage Limit','Discontinuities/Saturation',[660 155],{'Requested Voltage'}, ...
        {'UpperLimit',num2str(g.voltageLimit,17),'LowerLimit',num2str(-g.voltageLimit,17)}
    'Saturation Error','Math Operations/Sum',[635 405],{'Voltage Limit','Requested Voltage'}, ...
        {'Inputs','+-','Orientation','left'}
    'Anti-Windup Gain','Math Operations/Gain',[430 405],{'Saturation Error'}, ...
        {'Gain',num2str(g.antiWindupRate,17),'Orientation','left'}
    'Voltage','Sinks/Out1',[795 155],{'Voltage Limit'}, {'Port','1'}
    'Unsaturated Voltage','Sinks/Out1',[795 85],{'Requested Voltage'}, {'Port','2'}
});
wire(controller,'Arm Angle','Arm Rate Filter/1');
wire(controller,'Pendulum Angle','Pendulum Rate Filter/1');
end

function addRateFilter(block,position,rate,initialAngle)
add_block('built-in/Subsystem',block,'Position',position);
makeBlocks(block,{
    'Angle','Sources/In1',[25 55],{}, {'Port','1'}
    'Angle Error','Math Operations/Sum',[115 55],{'Angle','Filtered Angle'}, {'Inputs','+-'}
    'Filter Rate','Math Operations/Gain',[215 55],{'Angle Error'}, {'Gain',num2str(rate,17)}
    'Filtered Angle','Continuous/Integrator',[215 140],{'Filter Rate'}, ...
        {'InitialCondition',initialAngle,'Orientation','left'}
    'Angular Rate','Sinks/Out1',[340 55],{'Filter Rate'}, {'Port','1'}
});
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
