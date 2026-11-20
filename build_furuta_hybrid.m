function build_furuta_hybrid(source)
% Build a sampled energy controller with an actual Stateflow supervisor.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
p = furuta_parameters(source);
cfg = furuta_swingup_settings;
design = furuta_lqr_design(p);
poleDesign = furuta_pole_placement_design(p);
pidGains = furuta_pid_gains(p);
pidGains.voltageLimit = cfg.voltageLimit;
model = char("furuta_hybrid_"+source);
plant = char("furuta_plant_"+source);
if bdIsLoaded(model)
    error('FurutaHybrid:ModelOpen','Close %s before rebuilding.',model);
end
build_furuta_plant(source);
load_system(fullfile(root,'models',[plant '.slx']));
closePlant = onCleanup(@()close_system(plant,0));
new_system(model);
closeModel = onCleanup(@()close_system(model,0));
set_param(model,'Solver','ode45','RelTol','1e-8','AbsTol','1e-10', ...
    'MaxStep','0.001','StopTime','15');
w = get_param(model,'ModelWorkspace');
plantWorkspace = get_param(plant,'ModelWorkspace');
variables = whos(plantWorkspace);
for k=1:numel(variables)
    assignin(w,variables(k).name,getVariable(plantWorkspace,variables(k).name));
end
assignin(w,'cfg',cfg);
assignin(w,'initialState',[0;pi;0;0]);
assignin(w,'enableTime',0);
assignin(w,'disableTime',100);
add_block([plant '/Dynamics'],[model '/Plant Dynamics'],'Position',[825 80 975 170]);
add_furuta_hybrid_blocks(model,[350 260 530 375],[620 265 805 400], ...
    p,cfg,design,poleDesign,pidGains);
rows={
    'Controller Selection','Sources/Constant',[350 470],{}, {'Value','3','SampleTime','cfg.sampleTime'}
    'Controller Memory','Discrete/Unit Delay',[620 500],{'Hybrid Controller/5'}, {'InitialCondition','[0;0;0;-1;0]','SampleTime','cfg.sampleTime'}
    'Plant State','Continuous/Integrator',[1050 95],{'Plant Dynamics'}, {'InitialCondition','initialState'}
    'Sampled State','Discrete/Zero-Order Hold',[115 265],{'Plant State'}, {'SampleTime','cfg.sampleTime'}
    'Enable','Sources/Step',[40 430],{}, {'Time','enableTime','Before','0','After','1','SampleTime','cfg.sampleTime'}
    'Disable','Sources/Step',[40 505],{}, {'Time','disableTime','Before','0','After','1','SampleTime','cfg.sampleTime'}
    'Enabled','Math Operations/Sum',[200 440],{'Enable','Disable'}, {'Inputs','+-'}
    'Results','Signal Routing/Mux',[1260 100],{'Plant State','Hybrid Controller/1','Supervisor/1','Hybrid Controller/2','Hybrid Controller/3','Supervisor/2','Hybrid Controller/4','Controller Selection'}, {'Inputs','8'}
    'Flatten Results','Math Operations/Reshape',[1410 100],{'Results'}, {'OutputDimensionality','1-D array'}
    'States Voltage Mode Energy Angle Reference','Sinks/Out1',[1580 110],{'Flatten Results'}, {'Port','1'}
    'Pendulum Angle Scope','Sinks/Scope',[1000 300],{'Hybrid Controller/3'}, {}
    'Mode Scope','Sinks/Scope',[1000 405],{'Supervisor/1'}, {}
    'Voltage Scope','Sinks/Scope',[1000 510],{'Hybrid Controller/1'}, {}
};
for k=1:size(rows,1)
    xy=rows{k,3}; parameters=rows{k,5};
    add_block(['simulink/' rows{k,2}],[model '/' rows{k,1}], ...
        'Position',[xy xy+[70 40]],parameters{:});
end
for k=1:size(rows,1)
    inputs=rows{k,4};
    for j=1:numel(inputs)
        wire(model,inputs{j},sprintf('%s/%d',rows{k,1},j));
    end
end
wire(model,'Sampled State','Supervisor/1');
wire(model,'Enabled','Supervisor/2');
wire(model,'Controller Selection','Supervisor/3');
wire(model,'Controller Selection','Hybrid Controller/4');
wire(model,'Controller Memory','Hybrid Controller/5');
set_param([model '/Controller Selection'],'AttributesFormatString', ...
    sprintf('1 = PID/PD\n2 = Pole placement\n3 = LQR'));
wire(model,'Sampled State','Hybrid Controller/1');
wire(model,'Supervisor/1','Hybrid Controller/2');
wire(model,'Supervisor/2','Hybrid Controller/3');
wire(model,'Plant State','Plant Dynamics/1');
wire(model,'Hybrid Controller/1','Plant Dynamics/2');
set_param(model,'SimulationCommand','update');
save_system(model,fullfile(root,'models',[model '.slx']));
fprintf('Saved hybrid Stateflow model using %s parameters.\n',source);
end

function wire(system,from,to)
if ~contains(from,'/')
    from=[from '/1'];
end
add_line(system,from,to,'autorouting','on');
end
