function add_furuta_electrical_drive(sys,position)
% Switching bridge and physical DC motor, coupled to the fitted output torque.
add_block('built-in/Subsystem',sys,'Position',position,'BackgroundColor','[0.98 0.93 0.80]');
electrical = 'fl_lib/Electrical/';
mechanical = 'fl_lib/Mechanical/';
signal = 'simulink/';

add_block([signal 'Sources/In1'],[sys '/Voltage Command V'],'Position',[35 85 65 105]);
add_block([signal 'Sources/In1'],[sys '/Brake Command'],'Port','2','Position',[35 240 65 260]);
add_block([signal 'Sources/In1'],[sys '/Arm Speed rad_s'],'Port','3','Position',[35 415 65 435]);
add_block([signal 'Math Operations/Abs'],[sys '/Command Magnitude'],'Position',[100 60 130 90]);
gain(sys,'Duty Reference','5/drive.voltageLimit',[155 55 220 95]);
limitLogic(sys,'PWM Input Range',[250 55 305 95]);
add_block([signal 'Math Operations/Sign'],[sys '/Command Sign'],'Position',[100 145 130 175]);
gain(sys,'Reverse Logic','-5',[155 140 220 180]);
limitLogic(sys,'Reverse Input Range',[250 140 305 180]);
gain(sys,'Brake Logic','5',[155 230 220 270]);
gain(sys,'Gear Speed Ratio','drive.gearRatio',[155 395 220 435]);

duty = voltageInput(sys,'Duty Reference Source',[405 50 475 95]);
reverse = voltageInput(sys,'Reverse Source',[405 135 475 180]);
brake = voltageInput(sys,'Brake Source',[405 225 475 270]);
add_line(sys,'Voltage Command V/1','Command Magnitude/1','autorouting','on');
add_line(sys,'Command Magnitude/1','Duty Reference/1','autorouting','on');
add_line(sys,'Duty Reference/1','PWM Input Range/1','autorouting','on');
add_line(sys,'PWM Input Range/1','Duty Reference Source PS/1','autorouting','on');
add_line(sys,'Voltage Command V/1','Command Sign/1','autorouting','on');
add_line(sys,'Command Sign/1','Reverse Logic/1','autorouting','on');
add_line(sys,'Reverse Logic/1','Reverse Input Range/1','autorouting','on');
add_line(sys,'Reverse Input Range/1','Reverse Source PS/1','autorouting','on');
add_line(sys,'Brake Command/1','Brake Logic/1','autorouting','on');
add_line(sys,'Brake Logic/1','Brake Source PS/1','autorouting','on');

[~,switchingModes] = enumeration('ee.enum.ic.event');
discreteMode = switchingModes(~strcmp(switchingModes,'asynchronous'));
assert(isscalar(string(discreteMode)),'FurutaElectrical:PWM','Missing discrete PWM event mode.');
pwm = networkBlock(sys,'Controlled PWM Voltage',sprintf('ee_lib/Integrated Circuits/Controlled PWM\nVoltage'), ...
    [530 50 615 110],'F','drive.pwmFrequency','F_unit','Hz', ...
    'event_type',['ee.enum.ic.event.' char(discreteMode)], ...
    'sample_time','1/(200*drive.pwmFrequency)');
[~,freewheelModes] = enumeration('ee.enum.converters.freewheeling_mode');
twoDiodes = freewheelModes(startsWith(freewheelModes,'two') & ~contains(freewheelModes,'switch'));
assert(isscalar(string(twoDiodes)),'FurutaElectrical:Freewheel','Missing two-diode freewheel mode.');
bridge = networkBlock(sys,'L298N H-Bridge',sprintf('ee_lib/Semiconductors &\nConverters/Converters/H-Bridge'), ...
    [675 65 775 235],'Vm','drive.voltageLimit-drive.bridgeDrop', ...
    'freewheeling_mode',['ee.enum.converters.freewheeling_mode.' char(twoDiodes)]);
ground = networkBlock(sys,'Electrical Reference',[electrical 'Electrical Elements/Electrical Reference'], ...
    [625 315 665 355]);
solver = networkBlock(sys,'Electrical Solver','nesl_utility/Solver Configuration',[755 325 825 370], ...
    'UseLocalSolver','on','LocalSolverSampleTime','1/(200*drive.pwmFrequency)');
current = networkBlock(sys,'Current Sensor',[electrical 'Electrical Sensors/Current Sensor'],[835 55 900 95]);
voltage = networkBlock(sys,'Terminal Voltage Sensor',[electrical 'Electrical Sensors/Voltage Sensor'], ...
    [835 225 900 265]);
motor = networkBlock(sys,'GA25-370 DC Motor','ee_lib/Electromechanical/Brushed Motors/DC Motor', ...
    [970 95 1065 205],'Ra','drive.resistance','La','drive.armatureInductance', ...
    'Kv','drive.backEmfMotor','Kv_unit','V/(rad/s)', ...
    'J','drive.reflectedInertia/drive.gearRatio^2','J_unit','kg*m^2','lam','0');
speed = networkBlock(sys,'Shaft Speed Coupling',sprintf([mechanical 'Mechanical Sources/Ideal Angular\nVelocity Source']), ...
    [965 365 1050 425]);
caseRef = networkBlock(sys,'Motor Case Reference',sprintf([mechanical 'Rotational Elements/Mechanical\nRotational Reference']), ...
    [1105 395 1150 435]);
speedPS = networkBlock(sys,'Shaft Speed PS','nesl_utility/Simulink-PS Converter', ...
    [820 390 895 430],'Unit','rad/s','FilteringAndDerivatives','zero');
currentSignal = networkBlock(sys,'Current Signal','nesl_utility/PS-Simulink Converter', ...
    [1125 25 1200 65],'Unit','A');
voltageSignal = networkBlock(sys,'Voltage Signal','nesl_utility/PS-Simulink Converter', ...
    [1125 255 1200 295],'Unit','V');

connect(sys,duty.LConn(1),pwm.LConn(1));
connect(sys,duty.RConn(2),pwm.LConn(2));
connect(sys,pwm.RConn(1),bridge.LConn(1));
connect(sys,pwm.RConn(2),bridge.LConn(2));
connect(sys,reverse.LConn(1),bridge.LConn(3));
connect(sys,brake.LConn(1),bridge.LConn(4));
groundPorts = [duty.RConn(2) pwm.RConn(2) reverse.RConn(2) brake.RConn(2) ...
    solver.RConn(1) bridge.RConn(2) motor.RConn(1) voltage.RConn(2)];
for port = groundPorts
    connect(sys,ground.LConn(1),port);
end
connect(sys,bridge.RConn(1),current.LConn(1));
connect(sys,current.RConn(2),motor.LConn(1));
connect(sys,motor.LConn(1),voltage.LConn(1));
connect(sys,current.RConn(1),currentSignal.LConn(1));
connect(sys,voltage.RConn(1),voltageSignal.LConn(1));
connect(sys,speedPS.RConn(1),speed.RConn(1));
connect(sys,speed.LConn(1),motor.LConn(2));
connect(sys,caseRef.LConn(1),motor.RConn(2));
connect(sys,caseRef.LConn(1),speed.RConn(2));

% The practice fit provides independent effective constants. Return its
% output-side torque from the measured current. CAD carries reflected inertia.
gain(sys,'Effective Geared Torque','drive.torqueConstantOutput',[1245 95 1320 135]);
gain(sys,'Drive Viscous Loss','drive.viscousDamping',[1245 345 1320 385]);
add_block([signal 'Math Operations/Sum'],[sys '/Net Drive Torque'],'Inputs','+-', ...
    'IconShape','rectangular','Position',[1375 95 1400 145]);
add_block([signal 'Sinks/Out1'],[sys '/Drive Torque Nm'],'Position',[1475 105 1505 125]);
add_block([signal 'Sinks/Out1'],[sys '/Motor Current A'],'Port','2','Position',[1475 35 1505 55]);
add_block([signal 'Sinks/Out1'],[sys '/Motor Terminal Voltage V'],'Port','3','Position',[1475 265 1505 285]);
links = {
    'Arm Speed rad_s/1','Gear Speed Ratio/1'
    'Gear Speed Ratio/1','Shaft Speed PS/1'
    'Arm Speed rad_s/1','Drive Viscous Loss/1'
    'Current Signal/1','Effective Geared Torque/1'
    'Current Signal/1','Motor Current A/1'
    'Voltage Signal/1','Motor Terminal Voltage V/1'
    'Effective Geared Torque/1','Net Drive Torque/1'
    'Drive Viscous Loss/1','Net Drive Torque/2'
    'Net Drive Torque/1','Drive Torque Nm/1'
    };
for k=1:size(links,1)
    add_line(sys,links{k,1},links{k,2},'autorouting','on');
end
end

function handles = voltageInput(sys,name,pos)
converter = networkBlock(sys,[name ' PS'],'nesl_utility/Simulink-PS Converter', ...
    pos-[90 0 90 0],'Unit','V','FilteringAndDerivatives','zero');
handles = networkBlock(sys,name,sprintf('fl_lib/Electrical/Electrical Sources/Controlled Voltage\nSource'),pos);
connect(sys,converter.RConn(1),handles.RConn(1));
end

function handles = networkBlock(sys,name,library,position,varargin)
block = add_block(library,[sys '/' name],'Position',position,varargin{:});
handles = get_param(block,'PortHandles');
end

function gain(sys,name,value,position)
add_block('simulink/Math Operations/Gain',[sys '/' name],'Gain',value,'Position',position);
end

function limitLogic(sys,name,position)
add_block('simulink/Discontinuities/Saturation',[sys '/' name], ...
    'LowerLimit','0','UpperLimit','5','Position',position);
end

function connect(sys,from,to)
add_line(sys,from,to,'autorouting','on');
end
