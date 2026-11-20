function add_furuta_swingup_subsystem(path,position,p)
% Show the existing energy controller as three independently inspectable stages.
addSubsystem(path,position);
stage(path,{
    'Sampled State','Sources/In1',[30 140 60 160],{}
    'Joint Signals','Signal Routing/Demux',[110 155 120 290],{'Outputs','4'}
    'Swing-up Signals','Signal Routing/Mux',[1040 105 1050 215],{'Inputs','3'}
    'Voltage Energy and Request','Sinks/Out1',[1120 150 1150 170],{}
    },{
    'Sampled State/1','Joint Signals/1'
    'Swing-up Signals/1','Voltage Energy and Request/1'
    });
energy = [path '/Pendulum Energy'];
acceleration = [path '/Arm Acceleration Command'];
voltage = [path '/Torque and Motor Voltage'];
addSubsystem(energy,[235 120 385 215]);
addSubsystem(acceleration,[480 135 665 300]);
addSubsystem(voltage,[755 86 955 220]);
addEnergy(energy,p);
addAcceleration(acceleration,p);
addVoltage(voltage,p);
wire(path,{
    'Joint Signals/2','Pendulum Energy/1'
    'Joint Signals/4','Pendulum Energy/2'
    'Pendulum Energy/1','Arm Acceleration Command/1'
    'Pendulum Energy/2','Arm Acceleration Command/2'
    'Joint Signals/1','Arm Acceleration Command/3'
    'Joint Signals/3','Arm Acceleration Command/4'
    'Joint Signals/4','Arm Acceleration Command/5'
    'Sampled State/1','Torque and Motor Voltage/1'
    'Arm Acceleration Command/1','Torque and Motor Voltage/2'
    'Joint Signals/3','Torque and Motor Voltage/3'
    'Torque and Motor Voltage/1','Swing-up Signals/1'
    'Pendulum Energy/1','Swing-up Signals/2'
    'Torque and Motor Voltage/2','Swing-up Signals/3'
    });
set_param(energy,'ContentPreviewEnabled','off');
set_param(acceleration,'ContentPreviewEnabled','off');
set_param(voltage,'ContentPreviewEnabled','off');
end

function addEnergy(path,p)
Eup = num2str(p.m1*p.g*p.l1,17);
halfJ = num2str(0.5*(p.J1cm+p.m1*p.l1^2),17);
stage(path,{
    'Angle','Sources/In1',[30 60 60 80],{}
    'Rate','Sources/In1',[30 150 60 170],{'Port','2'}
    'Cosine','Math Operations/Trigonometric Function',[105 52 155 88],{'Operator','cos'}
    'Squared Rate','Math Operations/Math Function',[105 142 155 178],{'Operator','square'}
    'Potential Energy','Math Operations/Gain',[215 52 290 88],{'Gain',Eup}
    'Kinetic Energy','Math Operations/Gain',[215 142 290 178],{'Gain',halfJ}
    'Total Energy','Math Operations/Sum',[350 90 378 140],{'Inputs','++','IconShape','rectangular'}
    'Energy J','Sinks/Out1',[440 105 470 125],{}
    'Cos Angle','Sinks/Out1',[440 195 470 215],{'Port','2'}
    },{
    'Angle/1','Cosine/1'
    'Rate/1','Squared Rate/1'
    'Cosine/1','Potential Energy/1'
    'Cosine/1','Cos Angle/1'
    'Squared Rate/1','Kinetic Energy/1'
    'Kinetic Energy/1','Total Energy/1'
    'Potential Energy/1','Total Energy/2'
    'Total Energy/1','Energy J/1'
    });
end

function addAcceleration(path,p)
target = num2str(p.m1*p.g*p.l1,17);
stage(path,{
    'Energy J','Sources/In1',[30 45 60 65],{}
    'Cos Angle','Sources/In1',[30 112 60 132],{'Port','2'}
    'Arm Angle','Sources/In1',[30 205 60 225],{'Port','3'}
    'Arm Rate','Sources/In1',[30 275 60 295],{'Port','4'}
    'Pendulum Rate','Sources/In1',[30 350 60 370],{'Port','5'}
    'Energy Error','Math Operations/Bias',[115 38 185 72],{'Bias',['-' target '-cfg.energyOffset']}
    'Energy Gain','Math Operations/Gain',[223 38 290 72],{'Gain','cfg.energyGain'}
    'Direction Product','Math Operations/Product',[118 108 150 156],{}
    'Direction Gain','Math Operations/Gain',[185 110 250 150],{'Gain','cfg.signSlope'}
    'Smooth Direction','Math Operations/Trigonometric Function',[290 111 355 149],{'Operator','tanh'}
    'Energy Pumping','Math Operations/Product',[408 60 442 114],{}
    'Arm Centering','Math Operations/Gain',[195 194 275 235],{'Gain','cfg.armCenterGain'}
    'Arm Damping','Math Operations/Gain',[195 264 275 305],{'Gain','cfg.armDampingGain'}
    'Absolute Rate','Math Operations/Abs',[115 334 153 371],{}
    'Kick Rate Limit','Sources/Constant',[100 398 172 425],{'Value','cfg.kickRate'}
    'Near Rest','Logic and Bit Operations/Relational Operator',[220 346 254 399],{'Operator','<'}
    'Zero Energy','Sources/Constant',[100 473 150 499],{'Value','0'}
    'Below Zero Energy','Logic and Bit Operations/Relational Operator',[220 433 254 486],{'Operator','<'}
    'At Rest Near Hanging','Logic and Bit Operations/Logical Operator',[310 382 348 432],{'Operator','AND'}
    'Starting Push','Sources/Constant',[306 330 377 356],{'Value','cfg.kickAcceleration'}
    'No Push','Sources/Constant',[308 463 360 487],{'Value','0'}
    'Apply Starting Push','Signal Routing/Switch',[422 353 468 434],{'Criteria','u2 ~= 0'}
    'Requested Acceleration','Math Operations/Sum',[528 127 558 303],{'Inputs','+--+','IconShape','rectangular'}
    'Acceleration Limit','Discontinuities/Saturation',[623 192 690 236], ...
        {'LowerLimit','-cfg.accelerationLimit','UpperLimit','cfg.accelerationLimit'}
    'Arm Acceleration','Sinks/Out1',[750 204 780 224],{}
    },{
    'Energy J/1','Energy Error/1'
    'Energy Error/1','Energy Gain/1'
    'Cos Angle/1','Direction Product/1'
    'Pendulum Rate/1','Direction Product/2'
    'Direction Product/1','Direction Gain/1'
    'Direction Gain/1','Smooth Direction/1'
    'Energy Gain/1','Energy Pumping/1'
    'Smooth Direction/1','Energy Pumping/2'
    'Arm Angle/1','Arm Centering/1'
    'Arm Rate/1','Arm Damping/1'
    'Pendulum Rate/1','Absolute Rate/1'
    'Absolute Rate/1','Near Rest/1'
    'Kick Rate Limit/1','Near Rest/2'
    'Energy J/1','Below Zero Energy/1'
    'Zero Energy/1','Below Zero Energy/2'
    'Near Rest/1','At Rest Near Hanging/1'
    'Below Zero Energy/1','At Rest Near Hanging/2'
    'Starting Push/1','Apply Starting Push/1'
    'At Rest Near Hanging/1','Apply Starting Push/2'
    'No Push/1','Apply Starting Push/3'
    'Energy Pumping/1','Requested Acceleration/1'
    'Arm Centering/1','Requested Acceleration/2'
    'Arm Damping/1','Requested Acceleration/3'
    'Apply Starting Push/1','Requested Acceleration/4'
    'Requested Acceleration/1','Acceleration Limit/1'
    'Acceleration Limit/1','Arm Acceleration/1'
    });
end

function addVoltage(path,p)
stage(path,{
    'Sampled State','Sources/In1',[30 80 60 100],{}
    'Arm Acceleration','Sources/In1',[30 135 60 155],{'Port','2'}
    'Arm Rate','Sources/In1',[30 220 60 240],{'Port','3'}
    'Torque to Voltage','Math Operations/Gain',[335 94 425 142],{'Gain',num2str(p.R/(p.eta*p.N*p.Kt),17)}
    'Back EMF','Math Operations/Gain',[335 207 425 253],{'Gain',num2str(p.Ke*p.N,17)}
    'Requested Voltage','Math Operations/Sum',[485 115 515 210],{'Inputs','++','IconShape','rectangular'}
    'Voltage Limit','Discontinuities/Saturation',[570 105 640 149],{'LowerLimit','-cfg.voltageLimit','UpperLimit','cfg.voltageLimit'}
    'Motor Voltage V','Sinks/Out1',[705 117 735 137],{}
    'Unclipped Voltage V','Sinks/Out1',[705 208 735 228],{'Port','2'}
    },{
    'Arm Rate/1','Back EMF/1'
    'Torque to Voltage/1','Requested Voltage/1'
    'Back EMF/1','Requested Voltage/2'
    'Requested Voltage/1','Voltage Limit/1'
    'Requested Voltage/1','Unclipped Voltage V/1'
    'Voltage Limit/1','Motor Voltage V/1'
    });
torque = [path '/Inverse Mechanics'];
add_block('simulink/User-Defined Functions/MATLAB Function',torque,'Position',[140 72 270 158]);
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',torque);
fields = string(fieldnames(p));
values = arrayfun(@(name)p.(name),fields);
parameters = compose("p.%s = %.17g;",fields,values);
source = fileread(fullfile(fileparts(mfilename('fullpath')),'furuta_swingup_torque.m'));
chart.Script = char(strjoin(["function torque = inverse(state,acceleration)";parameters; ...
    "torque = furuta_swingup_torque(state,acceleration,p);";"end";string(source)],newline));
wire(path,{
    'Sampled State/1','Inverse Mechanics/1'
    'Arm Acceleration/1','Inverse Mechanics/2'
    'Inverse Mechanics/1','Torque to Voltage/1'
    });
end

function addSubsystem(path,position)
add_block('built-in/Subsystem',path,'Position',position,'BackgroundColor','[0.84 0.94 0.85]', ...
    'ContentPreviewEnabled','off');
end

function stage(path,blocks,connections)
for k=1:size(blocks,1)
    args = blocks{k,4};
    add_block(['simulink/' blocks{k,2}],[path '/' blocks{k,1}],'Position',blocks{k,3},args{:});
end
wire(path,connections);
end

function wire(path,connections)
for k=1:size(connections,1)
    add_line(path,connections{k,1},connections{k,2},'autorouting','on');
end
end
