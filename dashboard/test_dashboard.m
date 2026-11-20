function test_dashboard
% Smoke test of GUI, timer, external input, validation, and CSV export.
dashboardFolder = fileparts(mfilename('fullpath'));
output = fullfile(dashboardFolder,'output');
if ~isfolder(output), mkdir(output); end
app = FurutaDashboard('Visible','off');
cleanup = onCleanup(@()delete(app));
app.startDemo();
testClock = tic;
while height(app.Samples)<8 && toc(testClock)<30
    drawnow; pause(0.1);
end
app.stop();
assert(height(app.Samples)>5,'Demo timer produced too few samples.');
app.connectLive();
for k=1:30
    app.update(struct('t',k/20,'theta0',0.2*sin(k/20), ...
        'theta1',0.05*cos(k/20),'u',0.3,'mode','Stabilization'));
end
assert(height(app.Samples)==30);
assert(all(app.Samples.Source=="Live"));
assert(all(isnan(app.Samples.Omega0_rad_s)));
rejected = false;
try
    app.update(struct('t',1,'theta0',0,'theta1',0,'u',0,'mode','Swing-Up'));
catch e
    rejected = strcmp(e.identifier,'FurutaDashboard:Time');
end
assert(rejected,'Out-of-order sample was not rejected.');
app.exportCSV(fullfile(output,'dashboard_test.csv'));
saved = readtable(fullfile(output,'dashboard_test.csv'));
assert(height(saved)==30 && all(saved.MotorCommand==0.3));
app.Figure.Visible = 'on'; drawnow;
exportapp(app.Figure,fullfile(output,'dashboard_preview.png'));
app.resetDemo(); assert(isempty(app.Samples));
disp('DASHBOARD_TEST_PASSED');
clear cleanup app;
end
