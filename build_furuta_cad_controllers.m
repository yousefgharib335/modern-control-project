function build_furuta_cad_controllers(variant)
% Build the controller models around the shared switching electrical System.
arguments
    variant (1,1) string {mustBeMember(variant,["all","pid","pole_placement","lqr","hybrid"])} = "all"
end
root = fileparts(mfilename('fullpath'));
models = fullfile(root,'models');
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
plant = 'furuta_multibody_electrical';
variants = {'pid','pole_placement','lqr','hybrid'};
controllers = {'PID Controller','Pole Placement Controller','LQR Controller','Hybrid Controller'};
if ~bdIsLoaded(plant)
    load_system(fullfile(models,[plant '.slx']));
    closePlant = onCleanup(@()close_system(plant,0)); %#ok<NASGU>
end
legacyParameters = furuta_parameters("example");
for k = 1:numel(variants)
    if variant=="all" || variant==variants{k}
        buildModel(variants{k},controllers{k},plant,models,legacyParameters);
    end
end
end

function buildModel(variant,controller,plant,folder,legacyParameters)
model = ['furuta_' variant '_model'];
assert(~bdIsLoaded(model),'Close %s before rebuilding.',model);
new_system(model);
closeModel = onCleanup(@()close_system(model,0)); %#ok<NASGU>
config = copy(getActiveConfigSet(plant));
set_param(config,'Name','Electrical System Configuration');
attachConfigSet(model,config);
setActiveConfigSet(model,'Electrical System Configuration');
set_param(model,'StopTime','0.25','SimMechanicsOpenEditorOnUpdate','on', ...
    'Description',['Switching electrical closed-loop wiring checkout using the example controller gains. ' ...
    'Tune and validate the controllers on this plant before drawing performance conclusions.']);
workspace = get_param(model,'ModelWorkspace');
plantWorkspace = get_param(plant,'ModelWorkspace');
assignin(workspace,'p',getVariable(plantWorkspace,'p'));
drive = getVariable(plantWorkspace,'drive');
assignin(workspace,'drive',drive);
assignin(workspace,'furutaCoverImages',getVariable(plantWorkspace,'furutaCoverImages'));
isHybrid = strcmp(variant,'hybrid');
isPID = strcmp(variant,'pid');
initialState = [0;deg2rad(10);0;0];
if isHybrid
    initialState(2) = pi;
end
assignin(workspace,'initialState',initialState);
controllerPosition = [465 245 625 365];
if isHybrid
    addHybridCheckout(model,controllerPosition,workspace,legacyParameters);
else
    dataFolder = fullfile(fileparts(folder),'output','multibody');
    data = load(fullfile(dataFolder,'electrical_equilibrium_state_space.mat'));
    fit = load(fullfile(dataFolder,'electrical_upright_average_fit.mat'));
    pwm = furuta_pwm_compensation_config(fit,data.referenceVoltage,1/drive.pwmFrequency);
    controllerBlock = [model '/' controller];
    if isPID
        [g,poles] = furuta_electrical_pid_design(data,pwm);
        assignin(workspace,'pidGains',g);
        assignin(workspace,'pidAveragePoles',poles);
        add_furuta_pid_block(controllerBlock,controllerPosition,g);
    else
        plantMatrices = {data.upright.A,data.designB,data.upright.C,data.upright.D};
        if strcmp(variant,'lqr')
            % Keep nominal effort within the calibrated equivalent-voltage range.
            design = furuta_lqr_design(plantMatrices{:},3);
        else
            design = furuta_pole_placement_design(plantMatrices{:},[-4 -5 -6 -7]);
        end
        design.voltageLimit = pwm.equivalentVoltageLimit;
        design.initialState = initialState;
        assignin(workspace,'stateFeedbackDesign',design);
        addStateFeedback(controllerBlock,controllerPosition);
    end
    set_param(model,'StopTime','5','Description', ...
        [controller ' starting 10 degrees from upright with a short torque push at the pendulum joint.']);
end
% Keep joint targets local because Simscape treats them as non-tunable.
add_block([plant '/System'],[model '/System'],'Position',[865 70 1055 245]);
if ~isHybrid
    assignin(workspace,'pushTorque',4e-4);
    assignin(workspace,'pushStart',0.5);
    assignin(workspace,'pushDuration',0.05);
    addPendulumPush(model);
end
add_block('simulink/Math Operations/Reshape',[model '/Plant State'], ...
    'OutputDimensionality','Column vector (2-D)','Position',[1110 74 1195 114]);
add_block('simulink/Sources/In1',[model '/Test Voltage V'], ...
    'Position',[460 95 500 115]);
add_block('simulink/Sources/In1',[model '/Brake Command'],'Port','2', ...
    'Position',[690 177 730 197]);
add_block('simulink/Math Operations/Sum',[model '/Requested Voltage V'], ...
    'Inputs','++','Position',[715 112 750 155]);
add_block('simulink/Sinks/Out1',[model '/States'], ...
    'Port','1','Position',[1280 84 1320 104]);
add_block('simulink/Sinks/Out1',[model '/Controller Voltage V'], ...
    'Port','2','Position',[910 252 950 272]);
add_block('simulink/Sinks/Out1',[model '/Drive Torque Nm'], ...
    'Port','3','Position',[1150 165 1190 185]);
add_block('simulink/Sinks/Out1',[model '/Motor Current A'], ...
    'Port','4','Position',[1150 120 1190 140]);
terminalVoltagePort = 5 + isHybrid;
add_block('simulink/Sinks/Out1',[model '/Motor Terminal Voltage V'], ...
    'Port',num2str(terminalVoltagePort),'Position',[1150 205 1190 225]);
wire(model,'Test Voltage V/1','Requested Voltage V/1');
if ~isHybrid
    add_furuta_pwm_compensation([model '/Small Command Compensation'],[515 128 665 217],pwm);
    wire(model,[controller '/1'],'Small Command Compensation/1');
    wire(model,'Brake Command/1','Small Command Compensation/2');
    wire(model,'Small Command Compensation/1','Requested Voltage V/2');
    add_block('simulink/Sinks/Out1',[model '/Bridge Request V'], ...
        'Port','6','Position',[910 291 950 311]);
    wire(model,'Requested Voltage V/1','Bridge Request V/1');
else
    wire(model,[controller '/1'],'Requested Voltage V/2');
end
wire(model,'Brake Command/1','System/2');
wire(model,'System/1','Plant State/1');
wire(model,'System/2','Motor Current A/1');
wire(model,'System/3','Drive Torque Nm/1');
wire(model,'System/4','Motor Terminal Voltage V/1');
wire(model,'Plant State/1','States/1');
wire(model,[controller '/1'],'Controller Voltage V/1');

if isPID
    add_block('simulink/Signal Routing/Demux',[model '/Split State'], ...
        'Outputs','[1 1 2]','Position',[315 240 325 360]);
    add_block('simulink/Sinks/Terminator',[model '/Unused Rates'], ...
        'Position',[365 390 385 410]);
    wire(model,'Plant State/1','Split State/1');
    wire(model,'Split State/1',[controller '/1']);
    wire(model,'Split State/2',[controller '/2']);
    wire(model,'Split State/3','Unused Rates/1');
elseif isHybrid
    wire(model,'Plant State/1','Sampled State/1');
    wire(model,'Sampled State/1',[controller '/1']);
    wire(model,'Sampled State/1','Swing-Up/1');
    wire(model,'Supervisor/1',[controller '/2']);
    wire(model,'Supervisor/2',[controller '/3']);
    wire(model,'Controller Selection/1',[controller '/4']);
    wire(model,'Controller Memory/1',[controller '/5']);
    wire(model,'Swing-Up/1',[controller '/6']);
    wire(model,[controller '/5'],'Controller Memory/1');
    add_block('simulink/Sinks/Out1',[model '/Stateflow Mode'], ...
        'Port','5','Position',[925 485 965 505]);
    wire(model,'Supervisor/1','Stateflow Mode/1');
    % Suppress the injected voltage as well as the controller in Safe-Stop.
    add_block('simulink/Signal Routing/Switch',[model '/Drive Enabled'], ...
        'Threshold','0.5','Criteria','u2 >= Threshold','Position',[785 93 815 160]);
    add_block('simulink/Sources/Constant',[model '/Coast Voltage'], ...
        'Value','0','Position',[680 43 720 65]);
    wire(model,'Requested Voltage V/1','Drive Enabled/1');
    wire(model,'Supervisor/1','Drive Enabled/2');
    wire(model,'Coast Voltage/1','Drive Enabled/3');
    wire(model,'Drive Enabled/1','System/1');
else
    wire(model,'Plant State/1',[controller '/1']);
end
if ~isHybrid
    wire(model,'Requested Voltage V/1','System/1');
end
ports = get_param([model '/' controller],'PortHandles');
for n = 2:numel(ports.Outport)
    if isHybrid && n == 5
        continue
    end
    sink = sprintf('Unused Controller Output %d',n);
    y = 318 + 25*n;
    add_block('simulink/Sinks/Terminator',[model '/' sink], ...
        'Position',[680 y 700 y+20]);
    wire(model,sprintf('%s/%d',controller,n),[sink '/1']);
end
save_system(model,fullfile(folder,[model '.slx']));
fprintf('Built %s with the switching electrical System.\n',model);
end

function addPendulumPush(model)
push = [model '/Pendulum Push'];
add_block('built-in/Subsystem',push,'Position',[690 20 811 69], ...
    'BackgroundColor','[0.98 0.93 0.8]');
add_block('simulink/Sources/Step',[push '/Apply Push'],'Time','pushStart', ...
    'Before','0','After','pushTorque','Position',[40 40 70 70]);
add_block('simulink/Sources/Step',[push '/Remove Push'],'Time','pushStart+pushDuration', ...
    'Before','0','After','pushTorque','Position',[40 110 70 140]);
add_block('simulink/Math Operations/Sum',[push '/Torque Pulse'],'Inputs','+-', ...
    'Position',[130 61 155 119]);
add_block('simulink/Sinks/Out1',[push '/Torque Nm'],'Position',[220 80 250 100]);
wire(push,'Apply Push/1','Torque Pulse/1');
wire(push,'Remove Push/1','Torque Pulse/2');
wire(push,'Torque Pulse/1','Torque Nm/1');

system = [model '/System'];
mechanics = [system '/Pendulum Plant'];
add_block('simulink/Sources/In1',[system '/Push Torque Nm'], ...
    'Port','3','Position',[45 320 75 340]);
add_block('simulink/Sources/In1',[mechanics '/Push Torque Nm'], ...
    'Port','2','Position',[690 75 720 95]);
converter = [mechanics '/Push Torque Interface'];
add_block('nesl_utility/Simulink-PS Converter',converter, ...
    'Unit','N*m','FilteringAndDerivatives','zero','Position',[790 75 855 100]);
joint = [mechanics '/Pendulum Joint'];
set_param(joint,'TorqueActuationMode','InputTorque');
jointPorts = get_param(joint,'PortHandles');
converterPorts = get_param(converter,'PortHandles');
add_line(mechanics,converterPorts.RConn(1),jointPorts.LConn(2),'autorouting','on');
wire(mechanics,'Push Torque Nm/1','Push Torque Interface/1');
wire(system,'Push Torque Nm/1','Pendulum Plant/2');
wire(model,'Pendulum Push/1','System/3');
add_block('simulink/Sinks/Out1',[model '/Push Torque Nm'], ...
    'Port','7','Position',[1150 258 1190 278]);
wire(model,'Pendulum Push/1','Push Torque Nm/1');
systemMask = Simulink.Mask.get(system);
systemMask.Display = sprintf('%s\nport_label(''input'',3,''Push'');',systemMask.Display);
plantMask = Simulink.Mask.get(mechanics);
plantMask.Display = sprintf('%s\nport_label(''input'',2,''Push'');',plantMask.Display);
end

function addStateFeedback(block,position)
add_block('built-in/Subsystem',block,'Position',position);
add_block('simulink/Sources/In1',[block '/Joint State'],'Position',[30 65 60 85]);
add_block('simulink/Math Operations/Gain',[block '/State Feedback -K'], ...
    'Gain','-stateFeedbackDesign.K','Multiplication','Matrix(K*u)', ...
    'Position',[120 55 225 95]);
add_block('simulink/Discontinuities/Saturation',[block '/Voltage Limit'], ...
    'UpperLimit','stateFeedbackDesign.voltageLimit', ...
    'LowerLimit','-stateFeedbackDesign.voltageLimit','Position',[295 60 345 90]);
add_block('simulink/Sinks/Out1',[block '/Voltage'],'Position',[400 65 430 85]);
add_block('simulink/Sinks/Out1',[block '/Unsaturated Voltage'], ...
    'Port','2','Position',[400 125 430 145]);
wire(block,'Joint State/1','State Feedback -K/1');
wire(block,'State Feedback -K/1','Voltage Limit/1');
wire(block,'Voltage Limit/1','Voltage/1');
wire(block,'State Feedback -K/1','Unsaturated Voltage/1');
end

function addHybridCheckout(model,controllerPosition,workspace,legacyParameters)
cfg = furuta_swingup_settings;
pidGains = furuta_pid_gains(legacyParameters);
pidGains.voltageLimit = cfg.voltageLimit;
assignin(workspace,'cfg',cfg);
assignin(workspace,'enableTime',0);
assignin(workspace,'disableTime',100);
add_furuta_hybrid_blocks(model,[265 308 400 388],controllerPosition, ...
    legacyParameters,cfg,furuta_lqr_design(legacyParameters), ...
    furuta_pole_placement_design(legacyParameters),pidGains,true);
add_furuta_swingup_subsystem([model '/Swing-Up'],[275 185 405 245],legacyParameters);
add_block('simulink/Discrete/Zero-Order Hold',[model '/Sampled State'], ...
    'SampleTime','cfg.sampleTime','Position',[130 280 205 320]);
add_block('simulink/Sources/Constant',[model '/Controller Selection'], ...
    'Value','3','SampleTime','cfg.sampleTime','Position',[140 415 215 450], ...
    'AttributesFormatString',sprintf('1 = PID/PD\n2 = Pole placement\n3 = LQR'));
add_block('simulink/Discrete/Unit Delay',[model '/Controller Memory'], ...
    'InitialCondition','[0;0;0;-1;0]','SampleTime','cfg.sampleTime', ...
    'Position',[525 410 610 450]);
add_block('simulink/Sources/Step',[model '/Enable'], ...
    'Time','enableTime','Before','0','After','1','SampleTime','cfg.sampleTime', ...
    'Position',[35 520 90 555]);
add_block('simulink/Sources/Step',[model '/Disable'], ...
    'Time','disableTime','Before','0','After','1','SampleTime','cfg.sampleTime', ...
    'Position',[35 575 90 610]);
add_block('simulink/Math Operations/Sum',[model '/Enabled'], ...
    'Inputs','+-','Position',[140 542 180 585]);
wire(model,'Sampled State/1','Supervisor/1');
wire(model,'Enable/1','Enabled/1');
wire(model,'Disable/1','Enabled/2');
wire(model,'Enabled/1','Supervisor/2');
wire(model,'Controller Selection/1','Supervisor/3');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
