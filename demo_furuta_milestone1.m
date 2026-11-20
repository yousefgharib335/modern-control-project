function summary = demo_furuta_milestone1(showAnimation)
% Demonstrate open-loop fall from near upright on the CAD Simscape plant.
arguments
    showAnimation (1,1) logical = true
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'));
model = 'furuta_multibody';
modelFile = fullfile(root,'models',[model '.slx']);
load_system(modelFile);
if showAnimation
    open_system(model);
    open_system([model '/State Scope']);
end

p = furuta_cad_parameters;
% Rounded practice fits from synthetic coast-down and free-swing data.
p.armDamping = 1.1e-4;
p.pendulumDamping = 2.4e-5;
x0 = [0;deg2rad(1);0;0];
stopTime = 35;
input = Simulink.SimulationInput(model);
input = input.setVariable('p',p,'Workspace',model);
input = input.setVariable('initialState',x0,'Workspace',model);
input = input.setExternalInput([0 0;stopTime 0]);
if showAnimation
    explorer = 'on';
else
    explorer = 'off';
end
input = input.setModelParameter('StopTime',num2str(stopTime),'MaxStep','0.01', ...
    'SimMechanicsOpenEditorOnUpdate',explorer,'SaveTime','on','SaveOutput','on', ...
    'SaveFormat','Array','TimeSaveName','tout','OutputSaveName','yout');
simulation = sim(input);
time = simulation.tout;
state = simulation.yout;
assert(size(state,2)==4 && all(isfinite(state),'all'),'Expected four finite plant states.');
assert(max(abs(state(1,:).'-x0))<1e-6,'Unexpected initial conditions.');
angle = unwrap(state(:,2));
assert(max(angle)>pi,'The pendulum did not fall past hanging.');
errorFromHanging = atan2(sin(angle-pi),cos(angle-pi));
outsideBand = abs(rad2deg(errorFromHanging))>1 | abs(rad2deg(state(:,4)))>1;
lastOutside = find(outsideBand,1,'last');
assert(lastOutside<numel(time) && time(end)-time(lastOutside+1)>=2, ...
    'The pendulum did not settle to hanging for at least two seconds.');
settlingTime = time(lastOutside+1);

opts = odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',0.005);
[~,analytical] = ode45(@(~,x)furuta_cad_dynamics(x,0,p),time,x0,opts);
angleDifference = angle-unwrap(analytical(:,2));
maxPendulumDifference = max(abs(rad2deg(angleDifference)));
assert(maxPendulumDifference<0.01,'Simscape and analytical pendulum angles disagree.');

folder = fullfile(root,'output','multibody','milestone1');
if ~isfolder(folder)
    mkdir(folder);
end
trace = table(time,rad2deg(unwrap(state(:,1))),rad2deg(angle), ...
    rad2deg(state(:,3)),rad2deg(state(:,4)),rad2deg(unwrap(analytical(:,2))), ...
    zeros(size(time)),'VariableNames',{'time_s','arm_angle_deg','pendulum_angle_deg', ...
    'arm_rate_deg_s','pendulum_rate_deg_s','analytical_pendulum_deg','motor_torque_Nm'});
summary = table(rad2deg(angle(1)),max(rad2deg(angle)),rad2deg(angle(end)), ...
    settlingTime,maxPendulumDifference,p.armDamping,p.pendulumDamping, ...
    'VariableNames',{'initial_angle_deg','peak_angle_deg','final_angle_deg', ...
    'settling_time_s','max_analytical_angle_error_deg','assumed_arm_damping', ...
    'assumed_pendulum_damping'});
writetable(trace,fullfile(folder,'fall_from_upright.csv'));
writetable(summary,fullfile(folder,'summary.csv'));
disp(summary);
end
