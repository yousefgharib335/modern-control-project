function verify_furuta_pid(source)
% Verify nonlinear upright recovery and a repeatable voltage disturbance.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
p = furuta_parameters(source);
g = furuta_pid_gains(p);
model = char("furuta_pid_" + source);
load_system(fullfile(root,'models',[model '.slx']));
closeModel = onCleanup(@()close_system(model,0));
assert(getSimulinkBlockHandle([model '/PID Controller'])~=-1, ...
    'FurutaPID:Controller','PID Controller subsystem is missing.');
[A,B] = furuta_linearize(p);
% Linear closed loop includes the integral voltage and both derivative filters.
linear = zeros(7);
linear(1:4,1:4) = A;
linear(5,1) = g.armI;
linear(6,1) = g.filterRate; linear(6,6) = -g.filterRate;
linear(7,2) = g.filterRate; linear(7,7) = -g.filterRate;
feedback = [g.armP+g.armD*g.filterRate, g.pendulumP+g.pendulumD*g.filterRate, ...
    0,0,1,-g.armD*g.filterRate,-g.pendulumD*g.filterRate];
linear(1:4,:) = linear(1:4,:) + B*feedback;
poles = eig(linear);
assert(all(real(poles)<0),'FurutaPID:Stability','Filtered closed loop is unstable.');
output = fullfile(root,'output','pid');
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
        'FurutaPID:Capture','Pendulum left the tested upright region.');
    assert(abs(y(end,1))<deg2rad(0.1) && abs(y(end,2))<deg2rad(0.05), ...
        'FurutaPID:Recovery','Angles did not return near zero.');
    assert(max(abs(y(:,5)))<=g.voltageLimit+1e-8,'FurutaPID:Voltage','Voltage limit exceeded.');
    % Compare with an independently integrated controller and nonlinear plant.
    z0 = [initialState;0;initialState(1:2)];
    opts = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.005);
    [~,z] = ode45(@(time,state)closedLoop(time,state,p,g,pulse),t,z0,opts);
    assert(max(abs(y(:,1:4)-z(:,1:4)),[],'all')<2e-4, ...
        'FurutaPID:Reference','Simulink differs from independent integration.');
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
writetable(summary,fullfile(output,"pid_metrics_" + source + ".csv"));
disp(summary);
disp('Actual filtered closed-loop poles:'); disp(poles.');
disp(g);
disp('PID verification passed.');
end

function dz = closedLoop(t,z,p,g,pulse)
x = z(1:4);
raw = g.armP*x(1) + g.pendulumP*x(2) + z(5) + ...
    g.armD*g.filterRate*(x(1)-z(6)) + g.pendulumD*g.filterRate*(x(2)-z(7));
limited = min(g.voltageLimit,max(-g.voltageLimit,raw));
disturbance = pulse*(t>=4 && t<4.1);
voltage = min(g.voltageLimit,max(-g.voltageLimit,limited+disturbance));
dz = [furuta_dynamics(x,voltage,p); ...
    g.armI*x(1) + g.antiWindupRate*(limited-raw); ...
    g.filterRate*(x(1:2)-z(6:7))];
end
