function p = furuta_cad_parameters
% CAD-derived mechanical parameters in SI units for the selected rigid groups.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
file=fullfile(root,'cad','simscape-exports','plant-properties.json');
p=jsondecode(fileread(file));
end
