function add_furuta_motor_drive(subsystem,position,withDriver)
% Build a block-diagram DC motor and equivalent gearbox subsystem.
add_block('simulink/Ports & Subsystems/Subsystem',subsystem, ...
    'Position',position);
Simulink.SubSystem.deleteContents(subsystem);

add_block('simulink/Sources/In1',[subsystem '/Command V'],'Position',[35 95 65 115]);
add_block('simulink/Sources/In1',[subsystem '/Arm Rate rad_s'],'Port','2','Position',[35 245 65 265]);
add_block('simulink/Discontinuities/Saturation',[subsystem '/Supply Limit'], ...
    'UpperLimit','drive.voltageLimit','LowerLimit','-drive.voltageLimit','Position',[105 90 160 120]);
add_block('simulink/Math Operations/Sum',[subsystem '/Resistive Voltage'], ...
    'Inputs','+-','IconShape','rectangular','Position',[340 85 365 135]);
gain(subsystem,'Winding Conductance','1/drive.resistance',[410 88 480 132]);
gain(subsystem,'Gear Speed Ratio','drive.gearRatio',[105 183 185 227]);
gain(subsystem,'Back EMF','drive.backEmfMotor',[225 183 300 227]);
gain(subsystem,'Motor Torque Constant','drive.torqueConstantMotor',[530 88 605 132]);
gain(subsystem,'Gear Torque Gain','drive.gearRatio*drive.gearEfficiency',[660 88 745 132]);
gain(subsystem,'Drive Viscous Loss','drive.viscousDamping',[660 303 745 347]);
add_block('simulink/Math Operations/Sum',[subsystem '/Net Drive Torque'], ...
    'Inputs','+-','IconShape','rectangular','Position',[795 85 825 135]);
add_block('simulink/Sinks/Out1',[subsystem '/Drive Torque Nm'],'Position',[880 100 910 120]);
add_block('simulink/Sinks/Out1',[subsystem '/Current A'],'Port','2','Position',[540 35 570 55]);

links = {
    'Command V/1','Supply Limit/1'
    'Supply Limit/1','Resistive Voltage/1'
    'Arm Rate rad_s/1','Gear Speed Ratio/1'
    'Gear Speed Ratio/1','Back EMF/1'
    'Back EMF/1','Resistive Voltage/2'
    'Resistive Voltage/1','Winding Conductance/1'
    'Winding Conductance/1','Motor Torque Constant/1'
    'Winding Conductance/1','Current A/1'
    'Motor Torque Constant/1','Gear Torque Gain/1'
    'Gear Torque Gain/1','Net Drive Torque/1'
    'Arm Rate rad_s/1','Drive Viscous Loss/1'
    'Drive Viscous Loss/1','Net Drive Torque/2'
    'Net Drive Torque/1','Drive Torque Nm/1'
    };
for k = 1:size(links,1)
    add_line(subsystem,links{k,1},links{k,2},'autorouting','on');
end
if withDriver
    % Approximate each disabled PWM interval as zero winding current.
    add_block('simulink/Sources/In1',[subsystem '/Conducting Fraction'],'Port','3', ...
        'Position',[275 157 305 177]);
    add_block('simulink/Math Operations/Product',[subsystem '/Average Current'], ...
        'Position',[515 79 550 140]);
    set_param([subsystem '/Motor Torque Constant'],'Position',[590 88 635 132]);
    set_param([subsystem '/Current A'],'Position',[590 35 620 55]);
    delete_line(subsystem,'Winding Conductance/1','Motor Torque Constant/1');
    delete_line(subsystem,'Winding Conductance/1','Current A/1');
    add_line(subsystem,'Winding Conductance/1','Average Current/1','autorouting','on');
    add_line(subsystem,'Conducting Fraction/1','Average Current/2','autorouting','on');
    add_line(subsystem,'Average Current/1','Motor Torque Constant/1','autorouting','on');
    add_line(subsystem,'Average Current/1','Current A/1','autorouting','on');
end
end

function gain(subsystem,name,value,position)
add_block('simulink/Math Operations/Gain',[subsystem '/' name], ...
    'Gain',value,'Position',position);
end
