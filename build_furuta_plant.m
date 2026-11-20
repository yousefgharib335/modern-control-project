function build_furuta_plant(source)
% Rebuild the portable Simulink plant from standard library blocks.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
p = furuta_parameters(source);
model = char("furuta_plant_" + source);
folder = fullfile(root,'models');
if ~isfolder(folder)
    mkdir(folder);
end
if bdIsLoaded(model)
    error('FurutaPlant:ModelOpen','Close %s before rebuilding it.',model);
end
new_system(model);
closeModel = onCleanup(@()close_system(model,0));
set_param(model,'Solver','ode45','RelTol','1e-8','AbsTol','1e-10','StopTime','5');
w = get_param(model,'ModelWorkspace');
Jp = p.J1cm + p.m1*p.l1^2;
values = struct('Jp',Jp,'dJ',Jp-p.J1ax,'kCoupling',p.m1*p.L0*p.l1, ...
    'baseInertia',p.J0+p.m1*p.L0^2+p.J1ax,'gravityGain',p.m1*p.g*p.l1, ...
    'backEmfGain',p.Ke*p.N,'motorGain',p.eta*p.N*p.Kt/p.R,'b0',p.b0,'b1',p.b1);
for name = string(fieldnames(values)).'
    assignin(w,char(name),values.(name));
end

% Create the subsystems before wiring their external ports.
dynamics = addSubsystem(model,'Dynamics',[180 75 340 170]);
motor = addSubsystem(dynamics,'DC Motor',[190 45 320 100]);
inertia = addSubsystem(dynamics,'Mass Matrix',[190 135 320 190]);
forces = addSubsystem(dynamics,'Generalized Forces',[390 245 550 355]);
solve = addSubsystem(dynamics,'Acceleration Solve',[630 105 795 225]);
buildMotor(motor);
buildInertia(inertia);
buildForces(forces);
buildSolve(solve);

blocks(dynamics,{
    'State',        'Sources/In1',                   [35 160], {}, 'Port','1'
    'Voltage',      'Sources/In1',                   [35 55],  {}, 'Port','2'
    'Split State',  'Signal Routing/Demux',          [110 130],{'State'},'Outputs','4'
    'Assemble dx',  'Signal Routing/Mux',            [870 125],{'Split State/3','Split State/4','Acceleration Solve/1','Acceleration Solve/2'},'Inputs','4'
    'Derivative',   'Sinks/Out1',                    [970 145],{'Assemble dx'},'Port','1'
});
wire(dynamics,'Voltage','DC Motor/1');
wire(dynamics,'Split State/3','DC Motor/2');
wire(dynamics,'Split State/2','Mass Matrix/1');
wire(dynamics,'DC Motor','Generalized Forces/1');
wire(dynamics,'Split State/2','Generalized Forces/2');
wire(dynamics,'Split State/3','Generalized Forces/3');
wire(dynamics,'Split State/4','Generalized Forces/4');
wire(dynamics,'Mass Matrix/1','Acceleration Solve/1');
wire(dynamics,'Mass Matrix/2','Acceleration Solve/2');
wire(dynamics,'Generalized Forces/1','Acceleration Solve/3');
wire(dynamics,'Generalized Forces/2','Acceleration Solve/4');

blocks(model,{
    'Voltage_V',           'Sources/In1',           [50 135],{},'Port','1'
    'State',               'Continuous/Integrator',[430 100],{'Dynamics'},'InitialCondition','[0; pi+0.1; 0; 0]'
    'State_rad_and_rad_s',  'Sinks/Out1',            [580 105],{'State'},'Port','1'
});
wire(model,'State','Dynamics/1');
wire(model,'Voltage_V','Dynamics/2');
set_param(model,'SimulationCommand','update');
save_system(model,fullfile(folder,[model '.slx']));
fprintf('Saved %s\n',fullfile(folder,[model '.slx']));
end

function buildMotor(s)
blocks(s,{
    'Voltage',        'Sources/In1',              [35 45],{},'Port','1'
    'Arm Rate',       'Sources/In1',              [35 115],{},'Port','2'
    'Back EMF',       'Math Operations/Gain',     [140 110],{'Arm Rate'},'Gain','backEmfGain'
    'Net Voltage',    'Math Operations/Sum',      [255 50],{'Voltage','Back EMF'},'Inputs','+-'
    'Torque per Volt','Math Operations/Gain',     [375 50],{'Net Voltage'},'Gain','motorGain'
    'Torque',         'Sinks/Out1',               [510 55],{'Torque per Volt'},'Port','1'
});
end

function buildInertia(s)
blocks(s,{
    'Pendulum Angle', 'Sources/In1',                        [35 120],{},'Port','1'
    'Sine',           'Math Operations/Trigonometric Function',[140 55],{'Pendulum Angle'},'Operator','sin'
    'Cosine',         'Math Operations/Trigonometric Function',[140 205],{'Pendulum Angle'},'Operator','cos'
    'Sine Squared',   'Math Operations/Product',            [255 55],{'Sine','Sine'},'Inputs','**'
    'Angle Inertia',  'Math Operations/Gain',               [375 55],{'Sine Squared'},'Gain','dJ'
    'Base Inertia',   'Sources/Constant',                   [375 125],{},'Value','baseInertia'
    'Arm Inertia',    'Math Operations/Sum',                [505 60],{'Angle Inertia','Base Inertia'},'Inputs','++'
    'Coupling',       'Math Operations/Gain',               [505 205],{'Cosine'},'Gain','kCoupling'
    'M11',            'Sinks/Out1',                         [625 65],{'Arm Inertia'},'Port','1'
    'M12',            'Sinks/Out1',                         [625 210],{'Coupling'},'Port','2'
});
end

function buildForces(s)
blocks(s,{
    'Torque',         'Sources/In1',                        [35 45],{},'Port','1'
    'Pendulum Angle', 'Sources/In1',                        [35 135],{},'Port','2'
    'Arm Rate',       'Sources/In1',                        [35 255],{},'Port','3'
    'Pendulum Rate',  'Sources/In1',                        [35 445],{},'Port','4'
    'Sine',           'Math Operations/Trigonometric Function',[150 125],{'Pendulum Angle'},'Operator','sin'
    'Cosine',         'Math Operations/Trigonometric Function',[150 195],{'Pendulum Angle'},'Operator','cos'
    'Sine Cosine',    'Math Operations/Product',            [280 180],{'Sine','Cosine'},'Inputs','**'
    'Arm Friction',   'Math Operations/Gain',               [510 90],{'Arm Rate'},'Gain','b0'
    'Rate Product',   'Math Operations/Product',            [280 260],{'Arm Rate','Pendulum Rate'},'Inputs','**'
    'Cross Rate',     'Math Operations/Product',            [405 170],{'Sine Cosine','Rate Product'},'Inputs','**'
    'Coriolis',       'Math Operations/Gain',               [510 170],{'Cross Rate'},'Gain','2*dJ'
    'Pendulum Rate Squared','Math Operations/Product',      [280 410],{'Pendulum Rate','Pendulum Rate'},'Inputs','**'
    'Sine Pendulum Rate Squared','Math Operations/Product',  [405 260],{'Sine','Pendulum Rate Squared'},'Inputs','**'
    'Arm Centrifugal','Math Operations/Gain',               [510 260],{'Sine Pendulum Rate Squared'},'Gain','kCoupling'
    'Arm Rate Squared','Math Operations/Product',           [280 335],{'Arm Rate','Arm Rate'},'Inputs','**'
    'Sine Cosine Arm Rate Squared','Math Operations/Product',[405 345],{'Sine Cosine','Arm Rate Squared'},'Inputs','**'
    'Pendulum Centrifugal','Math Operations/Gain',          [510 345],{'Sine Cosine Arm Rate Squared'},'Gain','dJ'
    'Gravity',        'Math Operations/Gain',               [510 430],{'Sine'},'Gain','gravityGain'
    'Pendulum Friction','Math Operations/Gain',             [510 515],{'Pendulum Rate'},'Gain','b1'
    'Arm Net Torque', 'Math Operations/Sum',                [690 115],{'Torque','Arm Friction','Coriolis','Arm Centrifugal'},'Inputs','+--+'
    'Pendulum Net Torque','Math Operations/Sum',            [690 380],{'Pendulum Centrifugal','Gravity','Pendulum Friction'},'Inputs','++-'
    'F0',             'Sinks/Out1',                         [825 120],{'Arm Net Torque'},'Port','1'
    'F1',             'Sinks/Out1',                         [825 385],{'Pendulum Net Torque'},'Port','2'
});
end

function buildSolve(s)
% Solve the symmetric 2-by-2 mass-matrix system by its determinant.
blocks(s,{
    'M11',            'Sources/In1',          [35 65],{},'Port','1'
    'M12',            'Sources/In1',          [35 150],{},'Port','2'
    'F0',             'Sources/In1',          [35 280],{},'Port','3'
    'F1',             'Sources/In1',          [35 400],{},'Port','4'
    'M11 times Jp',   'Math Operations/Gain', [180 55],{'M11'},'Gain','Jp'
    'M12 Squared',    'Math Operations/Product',[180 130],{'M12','M12'},'Inputs','**'
    'Determinant',    'Math Operations/Sum',  [375 85],{'M11 times Jp','M12 Squared'},'Inputs','+-'
    'F0 times Jp',    'Math Operations/Gain', [180 220],{'F0'},'Gain','Jp'
    'M12 times F1',   'Math Operations/Product',[180 300],{'M12','F1'},'Inputs','**'
    'Arm Numerator',  'Math Operations/Sum',  [375 245],{'F0 times Jp','M12 times F1'},'Inputs','+-'
    'M11 times F1',   'Math Operations/Product',[180 380],{'M11','F1'},'Inputs','**'
    'M12 times F0',   'Math Operations/Product',[180 460],{'M12','F0'},'Inputs','**'
    'Pendulum Numerator','Math Operations/Sum',[375 405],{'M11 times F1','M12 times F0'},'Inputs','+-'
    'Arm Acceleration','Math Operations/Product',[550 235],{'Arm Numerator','Determinant'},'Inputs','*/'
    'Pendulum Acceleration','Math Operations/Product',[550 395],{'Pendulum Numerator','Determinant'},'Inputs','*/'
    'Arm Accel',      'Sinks/Out1',           [710 245],{'Arm Acceleration'},'Port','1'
    'Pendulum Accel', 'Sinks/Out1',           [710 405],{'Pendulum Acceleration'},'Port','2'
});
end

function subsystem = addSubsystem(system,name,position)
subsystem = [system '/' name];
add_block('simulink/Ports & Subsystems/Subsystem',subsystem,'Position',position);
Simulink.SubSystem.deleteContents(subsystem);
end

function blocks(system,definitions)
% Each row specifies the block name, library path, position, inputs and mask values.
for row = 1:size(definitions,1)
    [name,library,xy,~,parameter,value] = definitions{row,:};
    dimensions = [44 35];
    if endsWith(library,{'/Mux','/Demux'})
        dimensions = [10 78];
    end
    add_block(['simulink/' library],[system '/' name], ...
        'Position',[xy xy+dimensions],parameter,value);
end
for row = 1:size(definitions,1)
    name = definitions{row,1};
    inputs = definitions{row,4};
    for k = 1:numel(inputs)
        wire(system,inputs{k},sprintf('%s/%d',name,k));
    end
end
end

function wire(system,from,to)
if ~contains(from,'/')
    from = [from '/1'];
end
add_line(system,from,to,'autorouting','on');
end
