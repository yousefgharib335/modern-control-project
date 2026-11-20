function view_furuta_multibody
% Open the detailed CAD plant and its Simscape Multibody animation.
root=fileparts(mfilename('fullpath'));
model='furuta_multibody';
open_system(fullfile(root,'models',[model '.slx']));
set_param(model,'SimMechanicsOpenEditorOnUpdate','on');
sim(model);
disp('Detailed CAD animation is available in Multibody Explorer.');
end
