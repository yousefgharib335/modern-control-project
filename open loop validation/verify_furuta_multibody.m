function verify_furuta_multibody(showPlots)
% Compare the Simscape plant with general CAD rigid-body equations.
arguments
    showPlots (1,1) logical = true
end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'src','plant'));
p=furuta_cad_parameters;
model='furuta_multibody';
load_system(fullfile(root,'models',[model '.slx']));
cleanup=onCleanup(@()close_system(model,0));
output=fullfile(root,'output','multibody');
if ~isfolder(output)
    mkdir(output);
end
names=["hanging_free","hanging_torque","upright_small","hanging_damped"];
titles=["Free motion near hanging","Applied torque near hanging", ...
    "Release near upright","Damped motion near hanging"];
equilibria=[pi pi 0 pi];
initials={[0;pi+0.1;0;0],[0;pi+0.05;0;0],[0;0.01;0;0],[0;pi+0.1;0;0]};
stops=[2 0.75 0.15 2];
torques=[0 0.001 0 0];
summary=table();
if showPlots
    figure('Name','Open-loop validation','NumberTitle','off');
    overlays=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    title(overlays,'Open-loop validation against the nonlinear equations');
end
for test=1:numel(names)
    parameters=p;
    if test==4
        parameters.armDamping=0.0001;
        parameters.pendulumDamping=0.00002;
    end
    x0=initials{test};
    input=Simulink.SimulationInput(model);
    input=input.setVariable('p',parameters,'Workspace',model);
    input=input.setVariable('initialState',x0,'Workspace',model);
    input=input.setExternalInput([0 torques(test);stops(test) torques(test)]);
    input=input.setModelParameter('StopTime',num2str(stops(test),17), ...
        'SimMechanicsOpenEditorOnUpdate','off', ...
        'SaveTime','on','SaveOutput','on','SaveFormat','Array', ...
        'TimeSaveName','tout','OutputSaveName','yout');
    result=sim(input);
    t=result.tout;y=result.yout;
    assert(size(y,2)==4 && all(isfinite(y),'all'), ...
        'FurutaMultibody:Output','Expected four finite state outputs.');
    initialError=y(1,:).'-x0;
    initialError(1:2)=atan2(sin(initialError(1:2)),cos(initialError(1:2)));
    assert(max(abs(initialError))<1e-7,'FurutaMultibody:InitialState','Joint target or sensing convention differs.');
    opts=odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',0.0005);
    [~,reference]=ode45(@(~,x)furuta_cad_dynamics(x,torques(test),parameters),t,y(1,:).',opts);
    errors=y-reference;
    errors(:,1:2)=atan2(sin(errors(:,1:2)),cos(errors(:,1:2)));
    angleError=max(abs(errors(:,1:2)),[],'all');
    rateError=max(abs(errors(:,3:4)),[],'all');
    assert(angleError<1e-5 && rateError<1e-4,'FurutaMultibody:Mismatch', ...
        'Simscape and analytical model differ in %s: angle %.4g rad, rate %.4g rad/s.',names(test),angleError,rateError);
    energy=zeros(size(t));
    for k=1:numel(t)
        [~,energy(k)]=furuta_cad_dynamics(y(k,:).',torques(test),parameters);
    end
    if test==1
        assert(max(abs(energy-energy(1)))<1e-7,'FurutaMultibody:Energy','Unforced energy is inconsistent.');
    elseif test==3
        assert(abs(y(end,2))>abs(y(1,2)),'FurutaMultibody:Upright','Upright perturbation did not grow.');
    elseif test==4
        assert(energy(end)<energy(1) && max(diff(energy))<1e-9, ...
            'FurutaMultibody:Friction','Viscous damping energy check failed.');
    end
    data=array2table([t y reference],'VariableNames',{'Time_s','Theta0_rad','Theta1_rad', ...
        'Omega0_rad_s','Omega1_rad_s','ReferenceTheta0_rad','ReferenceTheta1_rad', ...
        'ReferenceOmega0_rad_s','ReferenceOmega1_rad_s'});
    writetable(data,fullfile(output,names(test)+".csv"));
    summary=[summary;table(names(test),angleError,rateError, ...
        'VariableNames',{'Test','MaxAngleError_rad','MaxRateError_rad_s'})];
    if showPlots
        ax=nexttile(overlays);
        compareResponse(ax,t,rad2deg(y(:,2)-equilibria(test)), ...
            rad2deg(reference(:,2)-equilibria(test)),titles(test), ...
            'Pendulum angle deviation (deg)','Simscape','Nonlinear equations');
    end
end

% Upright state-space representation of this same CAD mechanical model.
A=zeros(4);h=1e-20;
for k=1:4
    x=zeros(4,1);x(k)=1i*h;
    A(:,k)=imag(furuta_cad_dynamics(x,0,p))/h;
end
B=furuta_cad_dynamics(zeros(4,1),1,p)-furuta_cad_dynamics(zeros(4,1),0,p);
C=[eye(2) zeros(2)];D=zeros(2,1);
assert(max(real(eig(A)))>0,'FurutaMultibody:Linearization','Expected an unstable upright pole.');
x0=[0;0.001;0;0];
t=linspace(0,0.08,161).';
[~,nonlinear]=ode45(@(~,x)furuta_cad_dynamics(x,0,p),t,x0, ...
    odeset('RelTol',1e-11,'AbsTol',1e-13));
linear=zeros(size(nonlinear));
for k=1:numel(t)
    linear(k,:)=(expm(A*t(k))*x0).';
end
linearError=max(abs(nonlinear-linear),[],'all');
assert(linearError<1e-5,'FurutaMultibody:LinearModel','Small-signal state-space check failed.');
save(fullfile(output,'cad_state_space.mat'),'A','B','C','D','p','linearError');
writetable(array2table([t nonlinear linear],'VariableNames', ...
    {'Time_s','NonlinearTheta0','NonlinearTheta1','NonlinearOmega0','NonlinearOmega1', ...
    'LinearTheta0','LinearTheta1','LinearOmega0','LinearOmega1'}), ...
    fullfile(output,'upright_linear_comparison.csv'));
writetable(summary,fullfile(output,'validation_metrics.csv'));
if showPlots
    figure('Name','Upright linearization','NumberTitle','off');
    comparison=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    title(comparison,'Small release near upright');
    states=["Arm angle","Pendulum angle","Arm angular rate","Pendulum angular rate"];
    units=["Angle deviation (deg)","Angle deviation (deg)", ...
        "Angular rate (deg/s)","Angular rate (deg/s)"];
    for k=1:4
        compareResponse(nexttile(comparison),t,rad2deg(nonlinear(:,k)), ...
            rad2deg(linear(:,k)),states(k),units(k),'Nonlinear equations','Linearized model');
    end
end
disp(summary);
fprintf('Small-signal state-space max error %.3g.\n',linearError);
disp('Simscape, CAD analytical equations, energy and upright linearization checks passed.');
end

function compareResponse(ax,t,first,second,label,yLabel,firstName,secondName)
plot(ax,t,first,'-','LineWidth',1.5,'DisplayName',firstName);
hold(ax,'on');
plot(ax,t,second,'--','LineWidth',1.2,'DisplayName',secondName);
hold(ax,'off');
grid(ax,'on');
title(ax,label);
xlabel(ax,'Time (s)');
ylabel(ax,yLabel);
legend(ax,'Location','best');
end
