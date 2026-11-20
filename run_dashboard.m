% Run from the project folder. Closing the window cleans up the demo timer.
projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'dashboard'));
dashboard = FurutaDashboard;
dashboard.startDemo();
