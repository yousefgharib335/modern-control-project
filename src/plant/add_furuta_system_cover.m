function add_furuta_system_cover(model)
% Embed the CAD render and the component photos in the subsystem masks.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
renderFile = fullfile(root,'output','multibody','detailed_cad_preview.png');
render = imread(renderFile);
assembly = render(107:869,315:805,:);
photos = fullfile(root,'images');
system = [model '/System'];
bridge = [system '/H-Bridge'];
motor = [system '/DC Motor'];
plant = [system '/Pendulum Plant'];
images.system = fitCover(system,assembly,render(1,1,:),1);
images.bridge = productCover(bridge,fullfile(photos,'l298n-h-bridge.jpg'),75:1050,105:1020);
images.motor = productCover(motor,fullfile(photos,'ga25-370-dc-motor.png'),175:875,1:1001);
images.plant = fitCover(plant,assembly,render(1,1,:),1);
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'furutaCoverImages',images);
if hasVariable(workspace,'systemCoverRGB')
    evalin(workspace,'clear systemCoverRGB');
end

applyMask(system,'system','Furuta pendulum with switching L298N drive and CAD plant.', ...
    {'Command','Brake'},{'States','Current','Torque','Voltage'},{},{});
applyMask(bridge,'bridge','L298N bridge and PWM logic. Photo from Addicore.', ...
    {'Command','Brake'},{},{},{'+','-'});
applyMask(motor,'motor','GA25-370 geared motor and sensors. Photo from Most Electronics.', ...
    {'Arm Speed'},{'Current','Torque','Voltage'},{'+','-'},{});
applyMask(plant,'plant','CAD-derived pendulum and arm mechanics.', ...
    {'Torque'},{'States','Arm Speed'},{},{});
end

function cover = productCover(block,file,rows,columns)
pixels = imread(file);
cover = fitCover(block,pixels(rows,columns,:),uint8(255*ones(1,1,3)),3);
end

function cover = fitCover(block,pixels,background,pixelDensity)
position = get_param(block,'Position');
width = pixelDensity*(position(3)-position(1));
height = pixelDensity*(position(4)-position(2));
scale = min(0.60*width/size(pixels,2),0.88*height/size(pixels,1));
imageWidth = round(scale*size(pixels,2));
imageHeight = round(scale*size(pixels,1));
[x,y] = meshgrid(linspace(1,size(pixels,2),imageWidth),linspace(1,size(pixels,1),imageHeight));
cover = repmat(background,height,width);
top = floor((height-imageHeight)/2);
left = floor((width-imageWidth)/2);
for channel = 1:3
    cover(top+(1:imageHeight),left+(1:imageWidth),channel) = ...
        uint8(interp2(double(pixels(:,:,channel)),x,y,'linear'));
end
end

function applyMask(block,imageField,description,varargin)
mask = Simulink.Mask.get(block);
if isempty(mask)
    mask = Simulink.Mask.create(block);
end
mask.Description = description;
commands = {
    'covers = getVariable(get_param(bdroot(gcb),''ModelWorkspace''),''furutaCoverImages'');'
    sprintf('image(covers.%s,[0 0 1 1]);',imageField)
    'color(''black'');'
    };
types = {'input','output','lconn','rconn'};
for k = 1:numel(types)
    labels = varargin{k};
    for n = 1:numel(labels)
        commands{end+1,1} = sprintf('port_label(''%s'',%d,''%s'');',types{k},n,labels{n});
    end
end
mask.Display = strjoin(commands,newline);
set_param(block,'MaskIconUnits','normalized','OpenFcn','open_system(gcb,''force'')','ShowName','on');
end
