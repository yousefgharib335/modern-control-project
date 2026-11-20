function build_furuta_multibody(driveMode)
% Build the CAD-mass-property mechanical plant using Simscape Multibody blocks.
arguments
    driveMode (1,1) string {mustBeMember(driveMode,["torque","voltage","l298n","electrical","electrical_swingup"])} = "torque"
end
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
p=furuta_cad_parameters;
disp('Loaded CAD parameter snapshot.');
model='furuta_multibody';
voltageDriven=driveMode~="torque";
withDriver=driveMode=="l298n";
withSwingUp=driveMode=="electrical_swingup";
electrical=driveMode=="electrical" || withSwingUp;
if voltageDriven
    model='furuta_multibody_voltage';
    [drive,p]=furuta_assumed_motor_parameters(p);
end
if withDriver || electrical
    model='furuta_multibody_l298n';
    % Typical source and sink drops at 1 A, ST L298 datasheet Table 4.
    drive.bridgeDrop=1.35+1.20;
end
if electrical
    model='furuta_multibody_electrical';
    if withSwingUp
        model='furuta_multibody_electrical_swingup';
    end
    drive.armatureInductance=0.5e-3; % H, provisional until measured
    drive.pwmFrequency=1000; % Hz, provisional controller setting
    load_system('ee_lib');
    load_system('fl_lib');
end
if bdIsLoaded(model)
    error('FurutaMultibody:ModelOpen','Close %s before rebuilding.',model);
end
load_system('sm_lib');load_system('nesl_utility');
new_system(model);
cleanup=onCleanup(@()close_system(model,0));
set_param(model,'Solver','ode15s','RelTol','1e-8','AbsTol','1e-10', ...
    'MaxStep','0.001','StopTime','3','ModelReferenceMinAlgLoopOccurrences','on');
workspace=get_param(model,'ModelWorkspace');
assignin(workspace,'p',p);
assignin(workspace,'initialState',p.initialState);
if voltageDriven
    assignin(workspace,'drive',drive);
end
if electrical
    set_param(model,'Solver','ode23t','RelTol','1e-5','AbsTol','1e-8', ...
        'ZeroCrossAlgorithm','Adaptive','MaxStep',num2str(1/(20*drive.pwmFrequency),17));
end
folder=fullfile(root,'cad','simscape-exports');

mechanics = model;
if voltageDriven
    mechanics = [model '/Mechanical Plant'];
    offset = 240*withDriver;
    add_block('simulink/Ports & Subsystems/Subsystem',mechanics, ...
        'Position',[530 190 710 290]+[offset 0 offset 0], ...
        'BackgroundColor','[0.83 0.91 0.99]');
    Simulink.SubSystem.deleteContents(mechanics);
end
buildMechanics(mechanics,p,folder,voltageDriven);
if electrical
    connectElectricalPlant(model);
    group_furuta_electrical_system(model);
    if withSwingUp
        addpath(fullfile(root,'src','controllers'));
        add_furuta_electrical_swingup(model);
    end
elseif voltageDriven
    connectMotorAndPlant(model,withDriver,offset);
end
set_param(model,'SimscapeLogType','none','SimMechanicsOpenEditorOnUpdate','off');
disp('Compiling physical model.');
set_param(model,'SimulationCommand','update');
set_param(model,'SimMechanicsOpenEditorOnUpdate','on');
save_system(model,fullfile(root,'models',[model '.slx']));
disp('Saved CAD-derived Simscape Multibody plant.');
end

function connectElectricalPlant(model)
driveBlock = 'Electrical Motor and H-Bridge';
add_furuta_electrical_drive([model '/' driveBlock],[175 173 405 280]);
add_block('simulink/Sources/In1',[model '/Motor Voltage V'],'Position',[35 180 65 200]);
add_block('simulink/Sources/In1',[model '/Brake Command'],'Port','2','Position',[35 212 65 232]);
outputs = {'Angles and Rates','Motor Current A','Drive Torque Nm','Motor Terminal Voltage V'};
positions = {[825 204 855 224],[450 328 480 348],[455 120 485 140],[580 378 610 398]};
for k=1:numel(outputs)
    add_block('simulink/Sinks/Out1',[model '/' outputs{k}],'Port',num2str(k),'Position',positions{k});
end
wire(model,'Motor Voltage V/1',[driveBlock '/1'],'Requested voltage (V)');
wire(model,'Brake Command/1',[driveBlock '/2'],'');
wire(model,[driveBlock '/1'],'Mechanical Plant/1','Drive torque (N*m)');
wire(model,'Mechanical Plant/1','Angles and Rates/1','');
wire(model,[driveBlock '/1'],'Drive Torque Nm/1','');
wire(model,[driveBlock '/2'],'Motor Current A/1','');
wire(model,[driveBlock '/3'],'Motor Terminal Voltage V/1','');
plant = get_param([model '/Mechanical Plant'],'PortHandles');
actuator = get_param([model '/' driveBlock],'PortHandles');
from = get_param(plant.Outport(2),'Position');
to = get_param(actuator.Inport(3),'Position');
line = add_line(model,[from; 755 from(2); 755 455; 130 455; 130 to(2); to]);
set_param(line,'Name','Arm speed (rad/s)');
end

function buildMechanics(model,p,folder,voltageDriven)
add_block('sm_lib/Frames and Transforms/World Frame',[model '/World'],'Position',[90 80 135 125]);
add_block('sm_lib/Frames and Transforms/Rigid Transform',[model '/Assembly Mounting Frame'], ...
    'RotationMethod','RotationMatrix','RotationMatrix','p.mountRotation', ...
    'Position',[220 75 325 135]);
add_block('sm_lib/Utilities/Mechanism Configuration',[model '/Gravity'], ...
    'GravityVector','[0 0 -p.gravity]','GravityVectorUnits','m/s^2','Position',[90 180 160 225]);
add_block('nesl_utility/Solver Configuration',[model '/Solver'],'Position',[90 275 160 320]);
add_block('sm_lib/Joints/Revolute Joint',[model '/Arm Joint'], ...
    'TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on', ...
    'PositionTargetSpecify','on','PositionTargetValue','initialState(1)', ...
    'PositionTargetValueUnits','rad','VelocityTargetSpecify','on', ...
    'VelocityTargetValue','initialState(3)','VelocityTargetValueUnits','rad/s', ...
    'DampingCoefficient','p.armDamping','DampingCoefficientUnits','N*m/(rad/s)', ...
    'Position',[390 90 490 190]);
add_block('sm_lib/Frames and Transforms/Rigid Transform',[model '/Pendulum Hinge Frame'], ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset','p.hingeTranslation', ...
    'TranslationCartesianOffsetUnits','m','RotationMethod','RotationMatrix', ...
    'RotationMatrix','p.hingeRotation','Position',[660 140 770 215]);
add_block('sm_lib/Joints/Revolute Joint',[model '/Pendulum Joint'], ...
    'SensePosition','on','SenseVelocity','on', ...
    'PositionTargetSpecify','on','PositionTargetValue','initialState(2)', ...
    'PositionTargetValueUnits','rad','VelocityTargetSpecify','on', ...
    'VelocityTargetValue','initialState(4)','VelocityTargetValueUnits','rad/s', ...
    'DampingCoefficient','p.pendulumDamping','DampingCoefficientUnits','N*m/(rad/s)', ...
    'Position',[930 115 1030 215]);
world=get_param([model '/World'],'PortHandles');
mount=get_param([model '/Assembly Mounting Frame'],'PortHandles');
gravity=get_param([model '/Gravity'],'PortHandles');
solver=get_param([model '/Solver'],'PortHandles');
arm=get_param([model '/Arm Joint'],'PortHandles');
hinge=get_param([model '/Pendulum Hinge Frame'],'PortHandles');
pend=get_param([model '/Pendulum Joint'],'PortHandles');
physical(model,world.RConn(1),gravity.RConn(1));
physical(model,world.RConn(1),solver.RConn(1));
physical(model,world.RConn(1),mount.LConn(1));
physical(model,mount.RConn(1),arm.LConn(1));
physical(model,arm.RConn(1),hinge.LConn(1));
physical(model,hinge.RConn(1),pend.LConn(1));
if voltageDriven
    inertia=[model '/Reflected Motor and Gear Inertia'];
    add_block('sm_lib/Body Elements/Inertia',inertia, ...
        'InertiaType','Custom','Mass','0','MassUnits','kg', ...
        'MomentsOfInertia','drive.reflectedInertia*[0.5 0.5 1]', ...
        'MomentsOfInertiaUnits','kg*m^2','Position',[530 260 620 310]);
    inertiaPorts=get_param(inertia,'PortHandles');
    physical(model,arm.RConn(1),inertiaPorts.RConn(1));
end
disp('Connected gravity and both revolute joints.');
% Each source file keeps its own local CAD frame and COM inertia tensor.
bodyRows=zeros(1,3);
for k=1:numel(p.parts)
    part=p.parts(k);
    name=part.name;
    column=0;
    parent=mount.RConn(1);
    if strcmp(part.group,'arm')
        column=1;parent=arm.RConn(1);
    elseif strcmp(part.group,'pendulum')
        column=2;parent=pend.RConn(1);
    end
    x=200+column*500;
    y=450+bodyRows(column+1)*150;
    bodyRows(column+1)=bodyRows(column+1)+1;
    placement=[model '/' name ' Placement'];
    solid=[model '/' name];
    value=sprintf('p.parts(%d)',k);
    add_block('sm_lib/Frames and Transforms/Rigid Transform',placement, ...
        'TranslationMethod','Cartesian','TranslationCartesianOffset',[value '.translation'], ...
        'TranslationCartesianOffsetUnits','m','RotationMethod','RotationMatrix', ...
        'RotationMatrix',[value '.rotation'],'Position',[x y x+105 y+60]);
    geometry=fullfile(folder,part.mesh);
    assert(isfile(geometry),'FurutaMultibody:Geometry','Missing export %s.',geometry);
    I=part.inertia;
    fprintf('Adding CAD body %s\n',name);
    add_block('sm_lib/Body Elements/File Solid',solid, ...
        'ExtGeomFileName',geometry,'UnitType','Custom','ExtGeomFileUnits','m', ...
        'InertiaType','Custom', ...
        'Mass',num2str(part.mass,17),'MassUnits','kg', ...
        'CenterOfMass',mat2str(part.com(:).',17),'CenterOfMassUnits','m', ...
        'MomentsOfInertia',mat2str(diag(I).',17),'MomentsOfInertiaUnits','kg*m^2', ...
        'ProductsOfInertia',mat2str([I(2,3) I(1,3) I(1,2)],17), ...
        'ProductsOfInertiaUnits','kg*m^2','Position',[x+180 y x+290 y+65]);
    trans=get_param(placement,'PortHandles');
    body=get_param(solid,'PortHandles');
    physical(model,parent,trans.LConn(1));
    physical(model,trans.RConn(1),body.RConn(1));
end

add_block('nesl_utility/Simulink-PS Converter',[model '/Torque Interface'], ...
    'Unit','N*m','FilteringAndDerivatives','zero','Position',[240 355 310 400]);
input=get_param([model '/Torque Interface'],'PortHandles');
physical(model,input.RConn(1),arm.LConn(2));
names={'Arm Angle rad','Pendulum Angle rad','Arm Rate rad_s','Pendulum Rate rad_s'};
ports=[arm.RConn(2) pend.RConn(2) arm.RConn(3) pend.RConn(3)];
for k=1:4
    y=90+85*(k-1);
    block=[model '/' names{k}];
    unit='rad';
    if k>2
        unit='rad/s';
    end
    add_block('nesl_utility/PS-Simulink Converter',block,'Unit',unit, ...
        'Position',[1260 y 1330 y+40]);
    handles=get_param(block,'PortHandles');
    physical(model,ports(k),handles.LConn(1));
end
add_block('simulink/Signal Routing/Mux',[model '/State Vector'], ...
    'Inputs','4','Position',[1450 140 1460 325]);
for k=1:4
    add_line(model,[names{k} '/1'],sprintf('State Vector/%d',k),'autorouting','on');
end
add_block('simulink/Sinks/Out1',[model '/Angles and Rates'],'Position',[1580 185 1610 205]);
add_block('simulink/Sinks/Scope',[model '/State Scope'],'Position',[1580 285 1650 425]);
add_line(model,'State Vector/1','Angles and Rates/1','autorouting','on');
if voltageDriven
    add_block('simulink/Sources/In1',[model '/Drive Torque Nm'],'Position',[90 370 120 390]);
    add_block('simulink/Sinks/Out1',[model '/Arm Speed rad_s'],'Port','2', ...
        'Position',[1580 225 1610 245]);
    add_line(model,'Drive Torque Nm/1','Torque Interface/1','autorouting','on');
    add_line(model,'Arm Rate rad_s/1','Arm Speed rad_s/1','autorouting','on');
else
    add_block('simulink/Sources/In1',[model '/Motor Torque Nm'],'Position',[90 370 120 390]);
    add_line(model,'Motor Torque Nm/1','Torque Interface/1','autorouting','on');
end
configure_furuta_state_scope(model);
end

function connectMotorAndPlant(model,withDriver,offset)
motor = [model '/DC Motor and Gearbox'];
xOffset = [offset 0 offset 0];
add_furuta_motor_drive(motor,[200 185 375 265]+xOffset,withDriver);
set_param(motor,'BackgroundColor','[0.84 0.94 0.85]');
add_block('simulink/Sources/In1',[model '/Motor Voltage V'],'Position',[55 195 85 215]);
add_block('simulink/Sinks/Out1',[model '/Angles and Rates'], ...
    'Position',[865 204 895 224]+xOffset);
add_block('simulink/Sinks/Out1',[model '/Motor Current A'],'Port','2', ...
    'Position',[480 330 510 350]+xOffset);
add_block('simulink/Sinks/Out1',[model '/Drive Torque Nm'],'Port','3', ...
    'Position',[480 115 510 135]+xOffset);

if withDriver
    add_furuta_l298n_driver([model '/L298N Driver'],[160 172 345 250]);
    add_block('simulink/Sources/In1',[model '/Brake Command'],'Port','2', ...
        'Position',[55 220 85 240]);
    wire(model,'Motor Voltage V/1','L298N Driver/1','Requested voltage (V)');
    wire(model,'Brake Command/1','L298N Driver/2','');
    wire(model,'L298N Driver/1','DC Motor and Gearbox/1','On-state voltage (V)');
    wire(model,'L298N Driver/2','DC Motor and Gearbox/3','Conducting fraction');
else
    wire(model,'Motor Voltage V/1','DC Motor and Gearbox/1','Voltage (V)');
end
wire(model,'DC Motor and Gearbox/1','Mechanical Plant/1','Drive torque (N*m)');
wire(model,'DC Motor and Gearbox/1','Drive Torque Nm/1','');
wire(model,'DC Motor and Gearbox/2','Motor Current A/1','');
wire(model,'Mechanical Plant/1','Angles and Rates/1','');
plant = get_param([model '/Mechanical Plant'],'PortHandles');
motor = get_param(motor,'PortHandles');
from = get_param(plant.Outport(2),'Position');
to = get_param(motor.Inport(2),'Position');
feedback = add_line(model,[from; 760+offset from(2); 760+offset 410; ...
    145+offset 410; 145+offset to(2); to]);
set_param(feedback,'Name','Arm speed (rad/s)');
end

function wire(model,source,destination,label)
signal = add_line(model,source,destination,'autorouting','on');
if ~isempty(label)
    set_param(signal,'Name',label);
end
end

function physical(model,from,to)
add_line(model,from,to,'autorouting','on');
end
