function add_furuta_l298n_driver(path,position)
% Average L298N enable-PWM approximation with a separate brake command.
add_block('simulink/Ports & Subsystems/Subsystem',path, ...
    'Position',position,'BackgroundColor','[0.98 0.93 0.80]');
Simulink.SubSystem.deleteContents(path);
blocks = {
    'Voltage Command','Sources/In1',[35 100 65 120],{}
    'Brake Command','Sources/In1',[35 215 65 235],{'Port','2'}
    'Direction','Math Operations/Sign',[115 58 150 92],{}
    'Bridge On Voltage','Math Operations/Gain',[200 55 275 95], ...
        {'Gain','drive.voltageLimit-drive.bridgeDrop'}
    'Magnitude','Math Operations/Abs',[115 144 150 177],{}
    'Normalize Duty','Math Operations/Gain',[200 141 275 181],{'Gain','1/drive.voltageLimit'}
    'Limit Duty','Discontinuities/Saturation',[310 141 370 181],{'LowerLimit','0','UpperLimit','1'}
    'Brake Voltage','Sources/Constant',[310 26 360 46],{'Value','0'}
    'Brake Fraction','Sources/Constant',[310 255 360 275],{'Value','1'}
    'Select Voltage','Signal Routing/Switch',[445 45 490 105],{'Threshold','0.5','Criteria','u2 >= Threshold'}
    'Select Fraction','Signal Routing/Switch',[445 150 490 210],{'Threshold','0.5','Criteria','u2 >= Threshold'}
    'On State Voltage V','Sinks/Out1',[545 65 575 85],{}
    'Conducting Fraction','Sinks/Out1',[545 170 575 190],{'Port','2'}
    };
for k=1:size(blocks,1)
    add_block(['simulink/' blocks{k,2}],[path '/' blocks{k,1}], ...
        'Position',blocks{k,3},blocks{k,4}{:});
end
links = {
    'Voltage Command/1','Direction/1'
    'Direction/1','Bridge On Voltage/1'
    'Voltage Command/1','Magnitude/1'
    'Magnitude/1','Normalize Duty/1'
    'Normalize Duty/1','Limit Duty/1'
    'Brake Voltage/1','Select Voltage/1'
    'Brake Command/1','Select Voltage/2'
    'Bridge On Voltage/1','Select Voltage/3'
    'Brake Fraction/1','Select Fraction/1'
    'Brake Command/1','Select Fraction/2'
    'Limit Duty/1','Select Fraction/3'
    'Select Voltage/1','On State Voltage V/1'
    'Select Fraction/1','Conducting Fraction/1'
    };
for k=1:size(links,1)
    add_line(path,links{k,1},links{k,2},'autorouting','on');
end
end
