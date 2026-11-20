function group_furuta_electrical_system(model)
% Group the switching drive and pendulum while retaining the model I/O.
set_param([model '/Mechanical Plant'],'Name','Pendulum Plant');
blocks = find_system(model,'SearchDepth',1,'Type','Block');
handles = cell2mat(get_param(blocks,'Handle'));
Simulink.BlockDiagram.createSubsystem(handles,'Name','System','MakeNameUnique','off');
system = [model '/System'];
set_param(system,'Position',[295 125 540 345],'BackgroundColor','[0.83 0.91 0.99]');
ports = get_param(system,'PortHandles');
positionPorts(model,system,ports.Inport,'Inport',150);
positionPorts(model,system,ports.Outport,'Outport',680);
split_furuta_electrical_drive(model);
add_furuta_system_cover(model);
end

function positionPorts(model,system,ports,blockType,x)
inside = find_system(system,'SearchDepth',1,'BlockType',blockType);
outside = find_system(model,'SearchDepth',1,'BlockType',blockType);
isInput = strcmp(blockType,'Inport');
assert(numel(inside)==numel(outside) && numel(ports)==numel(inside), ...
    'FurutaElectrical:Interface','Grouping changed the root interface.');
for k=1:numel(outside)
    portNumber = get_param(outside{k},'Port');
    index = strcmp(get_param(inside,'Port'),portNumber);
    name = get_param(inside{index},'Name');
    set_param(outside{k},'Name',name);
    port = str2double(portNumber);
    xy = get_param(ports(port),'Position');
    set_param([model '/' name],'Position',[x xy(2)-10 x+30 xy(2)+10]);
    line = get_param(ports(port),'Line');
    delete_line(line);
    if isInput
        add_line(model,[name '/1'],sprintf('System/%d',port),'autorouting','on');
    else
        add_line(model,sprintf('System/%d',port),[name '/1'],'autorouting','on');
    end
end
end
