function configure_furuta_state_scope(model)
% Show one scalar state per tile, with one column per joint.
scope = [model '/State Scope'];
config = get_param(scope,'ScopeConfiguration');
config.NumInputPorts = '4';
config.LayoutDimensions = [2 2];
config.AxesScaling = 'Auto';
config.DataLoggingLimitDataPoints = false;
sources = {'Arm Angle rad','Arm Rate rad_s','Pendulum Angle rad','Pendulum Rate rad_s'};
titles = {'Arm angle','Arm angular velocity','Pendulum angle','Pendulum angular velocity'};
units = {'rad','rad/s','rad','rad/s'};
for k = 1:4
    config.ActiveDisplay = num2str(k);
    config.Title = titles{k};
    config.YLabel = units{k};
    add_line(model,[sources{k} '/1'],sprintf('State Scope/%d',k),'autorouting','on');
end
config.ActiveDisplay = '1';
end
