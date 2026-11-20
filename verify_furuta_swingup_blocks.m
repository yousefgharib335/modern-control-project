function verify_furuta_swingup_blocks
% Compare the saved visible Swing-Up chain against the MATLAB swing-up law.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','plant'),fullfile(root,'src','controllers'));
model = 'furuta_hybrid_model';
if ~bdIsLoaded(model)
    load_system(fullfile(root,'models',[model '.slx']));
    closeModel = onCleanup(@()close_system(model,0)); %#ok<NASGU>
end
cfg = getVariable(get_param(model,'ModelWorkspace'),'cfg');
p = furuta_parameters("example");
states = [
    0 pi 0 0
    0 pi 0 cfg.kickRate
    0 pi 0 0.5*cfg.kickRate
    0.3 2.1 1.4 -4.0
   -0.3 -2.1 -1.4 4.0
    0.7 0.2 -1.6 3.2
    0 0 25 -15
    0 0 -25 15
    ];
t = (0:size(states,1)-1).'*cfg.sampleTime;
harness = 'furuta_swingup_block_check';
assert(~bdIsLoaded(harness),'Close %s before verifying.',harness);
new_system(harness);
closeHarness = onCleanup(@()close_system(harness,0)); %#ok<NASGU>
assignin(get_param(harness,'ModelWorkspace'),'cfg',cfg);
add_block('simulink/Sources/In1',[harness '/State'],'PortDimensions','4');
add_block([model '/Swing-Up'],[harness '/Swing-Up'],'Position',[150 45 300 110]);
add_block('simulink/Sinks/Out1',[harness '/Signals'],'Position',[380 67 410 87]);
add_line(harness,'State/1','Swing-Up/1','autorouting','on');
add_line(harness,'Swing-Up/1','Signals/1','autorouting','on');
input = Simulink.SimulationInput(harness);
input = input.setExternalInput([t states]);
input = input.setModelParameter('Solver','FixedStepDiscrete','FixedStep',num2str(cfg.sampleTime,17), ...
    'StopTime',num2str(t(end),17),'SaveOutput','on','OutputSaveName','yout','SaveFormat','Array');
result = sim(input);
expected = zeros(size(states,1),3);
for k = 1:size(states,1)
    [expected(k,1),expected(k,2),expected(k,3)] = furuta_swingup(states(k,:).',p,cfg);
end
assert(isequal(size(result.yout),size(expected)),'Unexpected block outputs.');
errors = max(abs(result.yout-expected),[],1);
assert(max(errors)<1e-10,'FurutaSwingUp:BlockMismatch','The visible blocks differ from the swing-up function.');
fprintf('Swing-Up blocks matched the function for %d states. Max voltage error %.3g V.\n', ...
    size(states,1),errors(1));
end
