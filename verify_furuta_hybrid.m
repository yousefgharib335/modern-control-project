function verify_furuta_hybrid(source)
% Verify hanging-to-upright operation and supervisor transition priorities.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
cfg=furuta_swingup_settings;
model=char("furuta_hybrid_"+source);
load_system(fullfile(root,'models',[model '.slx']));
closeModel=onCleanup(@()close_system(model,0));
output=fullfile(root,'output','hybrid');
if ~isfolder(output)
    mkdir(output);
end
summary=table();
controllerNames=["PID_PD","PolePlacement","LQR"];
for controllerId=1:3
    for offset=[0 -0.1 0.1]
        initial=[0;pi+offset;0;0];
        input=Simulink.SimulationInput(model);
        input=input.setBlockParameter([model '/Controller Selection'],'Value',num2str(controllerId));
        input=input.setVariable('initialState',initial,'Workspace',model);
        input=input.setModelParameter('StopTime','15','SaveTime','on','SaveOutput','on', ...
            'SaveFormat','Array','TimeSaveName','tout','OutputSaveName','yout');
        result=sim(input);
        t=result.tout; y=result.yout;
        % y = x(1:4), voltage, mode, energy, wrapped angle, arm reference, raw voltage, selection.
        assert(all(isfinite(y),'all'),'FurutaHybrid:Finite','Nonfinite trajectory.');
        assert(y(1,6)==1 && any(y(:,6)==2) && ~any(y(:,6)==0), ...
            'FurutaHybrid:Sequence','Expected swing-up followed by stabilization.');
        first=find(y(:,6)==2,1);
        assert(all(y(first:end,6)==2),'FurutaHybrid:Capture','Capture was lost.');
        assert(abs(y(end,8))<deg2rad(0.1) && abs(y(end,1)-y(end,9))<deg2rad(0.1), ...
            'FurutaHybrid:Balance','Final angles are outside balance tolerances.');
        assert(max(abs(y(:,5)))<=cfg.voltageLimit+1e-9 && ...
            max(abs(y(:,1)))<cfg.armLimit,'FurutaHybrid:Limits','Simulation limit exceeded.');
        tag="hanging_"+string(round(offset*100))+"_"+source;
        if controllerId~=3
            tag=controllerNames(controllerId)+"_"+tag;
        end
        data=array2table([t y],'VariableNames',{'Time_s','Theta0_rad','Theta1_rad', ...
            'Omega0_rad_s','Omega1_rad_s','Voltage_V','Mode','Energy_J','UprightError_rad', ...
            'ArmReference_rad','RequestedVoltage_V','ControllerId'});
        writetable(data,fullfile(output,tag+".csv"));
        assert(all(y(:,11)==controllerId),'FurutaHybrid:Selection','Incorrect controller ID in output.');
        post=first:numel(t);
        outside=find(abs(y(post,8))>deg2rad(0.5) | abs(y(post,1)-y(post,9))>deg2rad(2),1,'last');
        recovery=0;
        if ~isempty(outside)
            recovery=t(post(min(outside+1,numel(post))))-t(first);
        end
        balanceRMS=sqrt(trapz(t(post),y(post,5).^2)/(t(end)-t(first)));
        metrics=table(controllerNames(controllerId),offset,t(first),max(abs(rad2deg(y(:,1)))),max(abs(y(:,5))), ...
            rad2deg(y(end,8)),rad2deg(y(end,1)-y(end,9)),recovery,balanceRMS, ...
            'VariableNames',{'Controller','InitialOffset_rad','CaptureTime_s','PeakArm_deg','PeakVoltage_V', ...
            'FinalPendulumError_deg','FinalArmReferenceError_deg','BalanceRecovery_s','BalanceRMSVoltage_V'});
        summary=[summary;metrics];
        if offset==0
            f=figure('Visible','off','Theme','light','Color','white','Position',[100 100 1050 850]);
            tiledlayout(f,4,1);
            nexttile;plot(t,rad2deg(y(:,8)));grid on;ylabel('Pendulum (deg)');
            nexttile;plot(t,rad2deg(y(:,[1 9])));grid on;ylabel('Arm (deg)');legend('Actual','Capture reference');
            nexttile;plot(t,y(:,5));grid on;ylabel('Voltage (V)');
            nexttile;stairs(t,y(:,6));grid on;ylabel('Mode');yticks([0 1 2]);
            yticklabels({'Safe stop','Swing-up',char(controllerNames(controllerId))});ylim([-0.2 2.2]);xlabel('Time (s)');
            plotName="swingup_"+source+".png";
            if controllerId~=3
                plotName=controllerNames(controllerId)+"_"+plotName;
            end
            exportgraphics(f,fullfile(output,plotName));
            close(f);
        end
    end
end
% Check the real chart with controlled inputs, independently of the plant.
verifySupervisor(model,cfg);
writetable(summary,fullfile(output,"hybrid_metrics_"+source+".csv"));
disp(summary);
disp('All three selected controllers passed swing-up, capture and balance. Supervisor checks passed.');
end

function verifySupervisor(sourceModel,cfg)
harness='furuta_supervisor_verification';
assert(~bdIsLoaded(harness),'FurutaHybrid:HarnessOpen','Close the supervisor verification model.');
new_system(harness);
cleanup=onCleanup(@()close_system(harness,0));
set_param(harness,'Solver','FixedStepDiscrete','FixedStep',num2str(cfg.sampleTime,17), ...
    'StopTime','0.035','SaveTime','on','SaveOutput','on','SaveFormat','Array', ...
    'TimeSaveName','tout','OutputSaveName','yout');
add_block([sourceModel '/Supervisor'],[harness '/Supervisor']);
add_block('simulink/Sources/In1',[harness '/State'],'Port','1','PortDimensions','4');
add_block('simulink/Sources/In1',[harness '/Enable'],'Port','2','PortDimensions','1');
add_block('simulink/Sources/In1',[harness '/Selection'],'Port','3','PortDimensions','1');
add_block('simulink/Signal Routing/Mux',[harness '/Mux'],'Inputs','2');
add_block('simulink/Sinks/Out1',[harness '/Out']);
add_line(harness,'State/1','Supervisor/1');
add_line(harness,'Enable/1','Supervisor/2');
add_line(harness,'Selection/1','Supervisor/3');
add_line(harness,'Supervisor/1','Mux/1');
add_line(harness,'Supervisor/2','Mux/2');
add_line(harness,'Mux/1','Out/1');
t=(0:7)'*cfg.sampleTime;
states=zeros(8,4);
states(1:2,2)=pi;
states(5,2)=deg2rad(45);
enabled=ones(8,1);enabled(7)=0;
input=Simulink.SimulationInput(harness);
for controllerId=1:3
result=sim(input.setExternalInput([t states enabled repmat(controllerId,8,1)]));
expected=[1;1;2;2;1;2;0;0];
assert(isequal(result.yout(:,1),expected),'FurutaHybrid:Transitions', ...
    'Supervisor capture, fallback, disable or latch sequence failed.');
end
% Both capture and travel-limit guards are true, and safe stop must win.
states=zeros(8,4);states(1:2,2)=pi;
states(3:end,1)=cfg.armLimit+0.1;
result=sim(input.setExternalInput([t states ones(8,1) 3*ones(8,1)]));
assert(all(result.yout(3:end,1)==0),'FurutaHybrid:Priority','Safe-stop priority failed.');
% A speed-limit violation must also stop swing-up.
states=zeros(8,4);states(1:2,2)=pi;states(3:end,3)=cfg.speedLimit+1;
result=sim(input.setExternalInput([t states ones(8,1) 3*ones(8,1)]));
assert(all(result.yout(3:end,1)==0),'FurutaHybrid:RateLimit','Speed-limit stop failed.');
states=zeros(8,4);states(1:2,2)=pi;
selection=3*ones(8,1);selection(3:end)=9;
result=sim(input.setExternalInput([t states ones(8,1) selection]));
assert(all(result.yout(3:end,1)==0),'FurutaHybrid:InvalidSelection','Invalid selector did not stop the controller.');
end
