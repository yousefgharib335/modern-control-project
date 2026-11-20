% Run verification from the project root; outputs go into dashboard/output/.
projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'dashboard'));
test_dashboard;
