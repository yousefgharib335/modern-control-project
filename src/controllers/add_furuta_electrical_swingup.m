function add_furuta_electrical_swingup(model)
% Connect the existing swing-up law to the switching electrical plant.
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'cfg',furuta_swingup_settings);
assignin(workspace,'initialState',[0;pi;0;0]);
p = furuta_parameters("example");
add_furuta_swingup_subsystem([model '/Swing-Up'],[235 428 375 492],p);
add_block('simulink/Discrete/Zero-Order Hold',[model '/Sampled States'], ...
    'SampleTime','cfg.sampleTime','Position',[105 444 178 476]);
add_block('simulink/Signal Routing/Selector',[model '/Swing-Up Voltage'], ...
    'IndexParamArray',{'1'},'Position',[410 439 460 481]);
add_block('simulink/Signal Routing/Variant Source',[model '/Drive Command'], ...
    'VariantControlMode','label','VariantControls',{'SwingUp','Manual'}, ...
    'LabelModeActiveChoice','SwingUp','Position',[485 195 545 300]);

plant = [model '/System'];
set_param(plant,'Position',[615 140 860 360]);
outputs = find_system(model,'SearchDepth',1,'BlockType','Outport');
for k=1:numel(outputs)
    pos = get_param(outputs{k},'Position');
    set_param(outputs{k},'Position',pos+[350 15 350 15]);
end
set_param([model '/Motor Voltage V'],'Position',[120 274 150 294]);
set_param([model '/Brake Command'],'Position',[120 295 150 315]);
delete_line(model,'Motor Voltage V/1','System/1');
add_line(model,'Motor Voltage V/1','Drive Command/2','autorouting','on');
add_line(model,'Drive Command/1','System/1','autorouting','on');
add_line(model,'Sampled States/1','Swing-Up/1','autorouting','on');
add_line(model,'Swing-Up/1','Swing-Up Voltage/1','autorouting','on');
add_line(model,'Swing-Up Voltage/1','Drive Command/1','autorouting','on');
feedback = add_line(model,'System/1','Sampled States/1','autorouting','on');
set_param(feedback,'Name','Joint states');
end
