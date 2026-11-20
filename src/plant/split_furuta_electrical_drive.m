function split_furuta_electrical_drive(model)
% Separate the bridge and motor while retaining their conserving connections.
system = [model '/System'];
drive = [system '/Electrical Motor and H-Bridge'];
if getSimulinkBlockHandle(drive)~=-1
bridgeBlocks = {
    'Command Magnitude','Duty Reference','PWM Input Range','Command Sign', ...
    'Reverse Logic','Reverse Input Range','Brake Logic', ...
    'Duty Reference Source','Duty Reference Source PS','Reverse Source','Reverse Source PS', ...
    'Brake Source','Brake Source PS','Controlled PWM Voltage','L298N H-Bridge', ...
    'Electrical Reference','Electrical Solver'};
motorBlocks = {
    'Gear Speed Ratio','Current Sensor','Terminal Voltage Sensor','GA25-370 DC Motor', ...
    'Shaft Speed Coupling','Motor Case Reference','Shaft Speed PS','Current Signal', ...
    'Voltage Signal','Effective Geared Torque','Drive Viscous Loss','Net Drive Torque'};
groupBlocks(drive,bridgeBlocks,'H-Bridge','Right');
groupBlocks(drive,motorBlocks,'DC Motor','Left');
Simulink.BlockDiagram.expandSubsystem(drive,'CreateArea','off');
end

bridge = [system '/H-Bridge'];
motor = [system '/DC Motor'];
plant = [system '/Pendulum Plant'];
set_param(bridge,'Position',[200 140 365 265],'BackgroundColor','[0.98 0.93 0.80]', ...
    'ContentPreviewEnabled','off');
set_param(motor,'Position',[495 140 665 265],'BackgroundColor','[0.84 0.94 0.85]', ...
    'ContentPreviewEnabled','off');
set_param(plant,'Position',[880 140 1060 245],'ContentPreviewEnabled','off');
renamePorts(bridge,'Inport',{'Command V','Brake'});
renamePorts(motor,'Inport',{'Arm Speed'});
renamePorts(motor,'Outport',{'Current','Torque','Voltage'});
renamePorts(bridge,'PMIOPort',{'Motor +','Motor -'});
renamePorts(motor,'PMIOPort',{'Motor +','Motor -'});
bridgePorts = get_param(bridge,'PortHandles');
motorPorts = get_param(motor,'PortHandles');
plantPorts = get_param(plant,'PortHandles');
assert(numel(bridgePorts.RConn)==2 && numel(motorPorts.LConn)==2, ...
    'FurutaElectrical:Connections','Expected two electrical terminals at each subsystem.');

command = movePort(system,'Motor Voltage V',45,portY(bridgePorts.Inport(1)),'Outport');
brake = movePort(system,'Brake Command',45,portY(bridgePorts.Inport(2)),'Outport');
states = movePort(system,'Angles and Rates',1170,portY(plantPorts.Outport(1)),'Inport');
current = movePort(system,'Motor Current A',775,portY(motorPorts.Outport(1))-40,'Inport');
torque = movePort(system,'Drive Torque Nm',840,315,'Inport');
voltage = movePort(system,'Motor Terminal Voltage V',775,portY(motorPorts.Outport(3)),'Inport');

sources = [command brake plantPorts.Outport motorPorts.Outport bridgePorts.RConn];
lines = arrayfun(@(port)get_param(port,'Line'),sources);
delete_line(unique(lines(lines~=-1)));
connections = [command bridgePorts.Inport(1); brake bridgePorts.Inport(2); ...
    bridgePorts.RConn(:) motorPorts.LConn(:); plantPorts.Outport(1) states; ...
    motorPorts.Outport(1) current; motorPorts.Outport(2) plantPorts.Inport(1); ...
    motorPorts.Outport(2) torque; motorPorts.Outport(3) voltage];
for k=1:size(connections,1)
    add_line(system,connections(k,1),connections(k,2),'autorouting','on');
end
from = get_param(plantPorts.Outport(2),'Position');
to = get_param(motorPorts.Inport(1),'Position');
feedback = add_line(system,[from; 1120 from(2); 1120 75; 440 75; 440 to(2); to]);
set_param(feedback,'Name','Arm speed (rad/s)');
end

function groupBlocks(parent,blocks,name,connectionSide)
paths = strcat([parent '/'],blocks);
handles = cell2mat(get_param(paths,'Handle'));
Simulink.BlockDiagram.createSubsystem(handles,'Name',name,'MakeNameUnique','off');
ports = find_system([parent '/' name],'SearchDepth',1,'BlockType','PMIOPort');
for k=1:numel(ports)
    set_param(ports{k},'Side',connectionSide);
end
end

function renamePorts(subsystem,blockType,names)
blocks = find_system(subsystem,'SearchDepth',1,'BlockType',blockType);
for k=1:numel(blocks)
    index = str2double(get_param(blocks{k},'Port'));
    set_param(blocks{k},'Name',names{index});
end
end

function handle = movePort(system,name,x,y,direction)
path = [system '/' name];
set_param(path,'Position',[x y-10 x+30 y+10]);
ports = get_param(path,'PortHandles');
handle = ports.(direction);
end

function y = portY(port)
position = get_param(port,'Position');
y = position(2);
end
