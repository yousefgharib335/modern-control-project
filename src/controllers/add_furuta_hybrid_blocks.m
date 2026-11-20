function add_furuta_hybrid_blocks(model,supervisorPosition,controllerPosition,p,cfg,design,poleDesign,pidGains,separateSwingUp)
% Build the shared Stateflow supervisor and MATLAB Function controller.
if nargin < 9
    separateSwingUp = false;
end
folder = fileparts(mfilename('fullpath'));
add_block('sflib/Chart',[model '/Supervisor'],'Position',supervisorPosition);
supervisor = find(sfroot,'-isa','Stateflow.Chart','Path',[model '/Supervisor']);
supervisor.ActionLanguage = 'MATLAB';
supervisor.ChartUpdate = 'DISCRETE';
supervisor.SampleTime = num2str(cfg.sampleTime,17);
stateData = Stateflow.Data(supervisor);
stateData.Name='state'; stateData.Scope='Input'; stateData.Port=1;
stateData.Props.Array.Size='[4 1]';
enableData = Stateflow.Data(supervisor);
enableData.Name='enabled'; enableData.Scope='Input'; enableData.Port=2;
selectionData = Stateflow.Data(supervisor);
selectionData.Name='controllerId'; selectionData.Scope='Input'; selectionData.Port=3;
modeData = Stateflow.Data(supervisor);
modeData.Name='mode'; modeData.Scope='Output'; modeData.Port=1;
referenceData = Stateflow.Data(supervisor);
referenceData.Name='armReference'; referenceData.Scope='Output'; referenceData.Port=2;

swing = Stateflow.State(supervisor);
swing.Position=[50 80 170 100];
swing.LabelString=sprintf('Swing_Up\nentry:\n mode=1; armReference=0;\nduring:\n mode=1;');
balance = Stateflow.State(supervisor);
balance.Position=[340 80 200 100];
balance.LabelString=sprintf('Stabilization\nentry:\n mode=2; armReference=state(1);\nduring:\n mode=2;');
stop = Stateflow.State(supervisor);
stop.Position=[190 310 200 80];
stop.LabelString=sprintf('Safe_Stop\nentry:\n mode=0; armReference=0;\nduring:\n mode=0;');
initial = Stateflow.Transition(supervisor);
initial.Destination=swing; initial.DestinationOClock=0;
initial.SourceEndPoint=[135 25]; initial.MidPoint=[135 50];

angle='atan2(sin(state(2)),cos(state(2)))';
predicted = sprintf('abs(%.17g*(%s)+%.17g*state(3)+%.17g*state(4))', ...
    design.K(2),angle,design.K(3),design.K(4));
pidPrediction = sprintf('abs(%.17g*(%s)+%.17g*state(3)+%.17g*state(4))', ...
    pidGains.pendulumP,angle,pidGains.armD,pidGains.pendulumD);
polePrediction = sprintf('abs(%.17g*(%s)+%.17g*state(3)+%.17g*state(4))', ...
    poleDesign.K(2),angle,poleDesign.K(3),poleDesign.K(4));
selectedPrediction = sprintf('((controllerId==1 && %s<%.17g) || (controllerId==2 && %s<%.17g) || controllerId==3)', ...
    pidPrediction,cfg.captureVoltage,polePrediction,cfg.captureVoltage);
capture = sprintf('[abs(%s)<%.17g && abs(state(4))<%.17g && abs(state(3))<%.17g && %s<%.17g && %s]', ...
    angle,cfg.captureAngle,cfg.captureRate,cfg.captureArmRate,predicted,cfg.captureVoltage,selectedPrediction);
fault = sprintf('[enabled<=0 || (controllerId~=1 && controllerId~=2 && controllerId~=3) || any(~isfinite(state)) || abs(state(1))>=%.17g || abs(state(3))>=%.17g || abs(state(4))>=%.17g]', ...
    cfg.armLimit,cfg.speedLimit,cfg.speedLimit);
transition(supervisor,swing,stop,fault,1,6,9);
transition(supervisor,balance,stop,fault,1,6,3);
transition(supervisor,swing,balance,capture,2,3,9);
release = sprintf('[abs(%s)>%.17g || abs(state(4))>%.17g]', ...
    angle,cfg.releaseAngle,cfg.releaseRate);
transition(supervisor,balance,swing,release,2,12,12);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [model '/Hybrid Controller'],'Position',controllerPosition);
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/Hybrid Controller']);
assignments=strings(0,1);
for name=string(fieldnames(p)).'
    assignments(end+1,1)=sprintf('p.%s=%.17g;',name,p.(name));
end
for name=string(fieldnames(cfg)).'
    assignments(end+1,1)=sprintf('cfg.%s=%.17g;',name,cfg.(name));
end
for name=string(fieldnames(pidGains)).'
    assignments(end+1,1)=sprintf('controllers.pid.%s=%s;',name,mat2str(pidGains.(name),17));
end
extraInput = "";
if separateSwingUp
    extraInput = ",swing";
end
chart.Script=char(strjoin([ ...
    "function [voltage,energy,angle,requestedVoltage,nextMemory]=controller(state,mode,armReference,controllerId,memory"+extraInput+")"; ...
    assignments; "controllers.lqrK="+string(mat2str(design.K,17))+";"; ...
    "controllers.poleK="+string(mat2str(poleDesign.K,17))+";"; ...
    "[voltage,energy,angle,requestedVoltage,nextMemory]=furuta_selected_control(state,mode,armReference,p,cfg,controllers,controllerId,memory"+extraInput+");"; ...
    "end"; ...
    string(fileread(fullfile(folder,'furuta_selected_control.m'))); ...
    string(fileread(fullfile(folder,'furuta_pid.m'))); ...
    string(fileread(fullfile(folder,'furuta_pole_placement.m'))); ...
    string(fileread(fullfile(folder,'furuta_hybrid_control.m'))); ...
    string(fileread(fullfile(folder,'furuta_swingup.m'))); ...
    string(fileread(fullfile(folder,'furuta_swingup_torque.m'))); ...
    string(fileread(fullfile(folder,'furuta_lqr.m')))],newline));
end

function transition(chart,from,to,label,order,sourceClock,destClock)
t=Stateflow.Transition(chart);
t.Source=from; t.Destination=to;
t.SourceOClock=sourceClock; t.DestinationOClock=destClock;
t.LabelString=label; t.ExecutionOrder=order;
end

