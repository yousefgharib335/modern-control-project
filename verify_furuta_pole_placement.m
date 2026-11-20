function verify_furuta_pole_placement(source)
% Verify nonlinear upright recovery and a repeatable voltage disturbance.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
p = furuta_parameters(source);
g = furuta_pole_placement_design(p);
model = char("furuta_pole_placement_" + source);
load_system(fullfile(root,'models',[model '.slx']));
closeModel = onCleanup(@()close_system(model,0));
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/Pole Placement Controller']);
assert(~isempty(chart),'FurutaPolePlacement:Function','Controller block is missing.');
poles = eig(g.A-g.B*g.K);
assert(all(real(poles)<0),'FurutaPolePlacement:Stability','Closed loop is unstable.');
output = fullfile(root,'output','pole-placement');
if ~isfolder(output)
    mkdir(output);
end
summary = table();
for test = 1:2
    initialState = g.initialState;
    pulse = 0;
    label = "initial_5deg";
    if test == 2
        initialState = zeros(4,1);
        pulse = 1;
        label = "voltage_disturbance";
    end
    input = Simulink.SimulationInput(model);
    input = input.setVariable('initialState',initialState,'Workspace',model);
    input = input.setVariable('disturbanceVoltage',pulse,'Workspace',model);
    input = input.setModelParameter('StopTime','12','SaveTime','on', ...
        'SaveOutput','on','SaveFormat','Array','TimeSaveName','tout','OutputSaveName','yout');
    result = sim(input);
    t = result.tout; y = result.yout;
    assert(all(isfinite(y),'all') && max(abs(y(:,2)))<deg2rad(20), ...
        'FurutaPolePlacement:Capture','Pendulum left the tested upright region.');
    assert(abs(y(end,1))<deg2rad(0.1) && abs(y(end,2))<deg2rad(0.05), ...
        'FurutaPolePlacement:Recovery','Angles did not return near zero.');
    assert(max(abs(y(:,5)))<=g.voltageLimit+1e-8,'FurutaPolePlacement:Voltage','Voltage limit exceeded.');
    % Compare with an independently integrated controller and nonlinear plant.
    z0 = initialState;
    opts = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.005);
    [~,z] = ode45(@(time,state)closedLoop(time,state,p,g,pulse),t,z0,opts);
    assert(max(abs(y(:,1:4)-z(:,1:4)),[],'all')<2e-4, ...
        'FurutaPolePlacement:Reference','Simulink differs from independent integration.');
    data = array2table([t,y],'VariableNames',{'Time_s','Theta0_rad','Theta1_rad', ...
        'Omega0_rad_s','Omega1_rad_s','AppliedVoltage_V','RequestedVoltage_V'});
    writetable(data,fullfile(output,label + "_" + source + ".csv"));
    after = 0;
    if test == 2
        after = 4.1;
    end
    outside = find(t>=after & (abs(y(:,2))>deg2rad(0.5) | abs(y(:,1))>deg2rad(2)),1,'last');
    settling = 0;
    if ~isempty(outside)
        settling = t(min(outside+1,numel(t)))-after;
    end
    metrics = table(label,max(abs(rad2deg(y(:,2)))),max(abs(rad2deg(y(:,1)))), ...
        max(abs(y(:,5))),sqrt(trapz(t,y(:,5).^2)/(t(end)-t(1))), ...
        trapz(t,abs(y(:,2))),settling, ...
        'VariableNames',{'Test','PeakPendulum_deg','PeakArm_deg','PeakVoltage_V', ...
        'RMSVoltage_V','PendulumIAE_rad_s','Recovery_s'});
    summary = [summary;metrics];
    fig = figure('Visible','off','Color','white','Theme','light','Position',[100 100 1000 720]);
    tiles = tiledlayout(fig,3,1);
    title(tiles,strrep(label + " | " + source + " parameters","_"," "));
    nexttile; plot(t,rad2deg(y(:,2)),'LineWidth',1.4); grid on;
    ylabel('Pendulum (deg)');
    nexttile; plot(t,rad2deg(y(:,1)),'LineWidth',1.4); grid on;
    ylabel('Arm (deg)');
    nexttile; plot(t,y(:,5),'LineWidth',1.4); grid on;
    ylabel('Motor voltage (V)'); xlabel('Time (s)');
    exportgraphics(fig,fullfile(output,label + "_" + source + ".png"));
    close(fig);
end
writetable(summary,fullfile(output,"pole_placement_metrics_" + source + ".csv"));
disp(summary);
disp('Closed-loop poles:'); disp(poles.');
disp(g);
disp('Pole-placement verification passed.');
end

function dz = closedLoop(t,z,p,g,pulse)
x = z(1:4);
raw = -g.K*x;
limited = min(g.voltageLimit,max(-g.voltageLimit,raw));
disturbance = pulse*(t>=4 && t<4.1);
voltage = min(g.voltageLimit,max(-g.voltageLimit,limited+disturbance));
dz = furuta_dynamics(x,voltage,p);
end
