function file = rebuild_trayectory_model(varargin)
% REBUILD_TRAYECTORY_MODEL  Recrea Trayectory_Model.slx con la version de MATLAB actual.
%
%   Trayectory_Model.slx se guardo con R2023b y no abre en R2023a. Esta funcion
%   lo vuelve a construir bloque por bloque (mismas posiciones, parametros,
%   conexiones y el codigo exacto de los 6 bloques MATLAB Function, tomado de
%   simulink/trayectory_blocks/) y lo guarda en tu version.
%
%   Ejecutar desde la carpeta del proyecto (donde esta setup_custom_model.m):
%
%     rebuild_trayectory_model                         % copia fiel + setup_custom_model
%     rebuild_trayectory_model('Camera','synthetic')   % sin webcam: imagen negra 720x1280 a 30 fps
%     rebuild_trayectory_model('ConnectTrigger',true)  % conecta startCamera -> Trigger (ver nota)
%     rebuild_trayectory_model('Visualization',false)  % omite el bloque de figuras FOMO
%     rebuild_trayectory_model('SetupNative',false)    % no compila/enlaza el DLL al terminar
%     rebuild_trayectory_model('Name','Trayectory_Model_R2023a')
%
%   Si ya existe <Name>.slx en la carpeta, se mueve a backups/original_models/
%   antes de guardar el nuevo (no se borra nada).
%
%   NOTA sobre el modelo original: la entrada Trigger de "Image Batch Capture"
%   esta conectada a una constante false y la salida startCamera de
%   "Trayectory SM" no esta conectada, asi que la captura nunca se dispara.
%   La copia fiel conserva eso. Con 'ConnectTrigger',true se conecta
%   startCamera -> Trigger a traves de un Unit Delay de 0.05 s (rompe el lazo
%   algebraico startCamera -> ImgCaptured -> nextPoint).

p = inputParser;
addParameter(p,'Name','Trayectory_Model',@(s) ischar(s) || isstring(s));
addParameter(p,'Camera','webcam',@(s) any(strcmpi(s,{'webcam','synthetic'})));
addParameter(p,'ConnectTrigger',false,@(x) islogical(x) || isnumeric(x));
addParameter(p,'Visualization',true,@(x) islogical(x) || isnumeric(x));
addParameter(p,'SetupNative',true,@(x) islogical(x) || isnumeric(x));
parse(p,varargin{:});
o = p.Results;
name = char(o.Name);

root = fileparts(mfilename('fullpath'));
assert(isfile(fullfile(root,'setup_custom_model.m')), 'EI:Rebuild', ...
    'Copia rebuild_trayectory_model.m a la carpeta raiz del proyecto (junto a setup_custom_model.m).');
src = fullfile(root,'simulink','trayectory_blocks');
assert(isfolder(src), 'EI:Rebuild', 'Falta la carpeta %s con el codigo de los bloques.', src);
assert(license('test','Simulink') == 1, 'EI:Rebuild', 'Se necesita Simulink.');
target = fullfile(root,[name '.slx']);

if bdIsLoaded(name), close_system(name,0); end
if isfile(target)
    backup = fullfile(root,'backups','original_models');
    if ~isfolder(backup), mkdir(backup); end
    stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
    moved = fullfile(backup,[name '_' stamp '.slx']);
    movefile(target, moved);
    fprintf('Modelo existente respaldado en: %s\n', moved);
end

new_system(name);
m = @(b) [name '/' b];

%% ---------------- Configuracion (igual que configSet0 del original) ----------------
set_param(name, 'StartTime','0.0', 'StopTime','inf', 'SolverName','VariableStepAuto', ...
    'EnableMultiTasking','off', 'AutoInsertRateTranBlk','off', ...
    'SingleTaskRateTransMsg','none', 'MultiTaskRateTransMsg','error', ...
    'AlgebraicLoopMsg','warning', 'SaveFormat','Dataset', 'SaveOutput','on', ...
    'SaveTime','on', 'SaveState','off', 'SignalLogging','on', 'DSMLogging','on', ...
    'ReturnWorkspaceOutputs','on', 'SimTargetLang','C', 'SimCompilerOptimization','off', ...
    'SimCtrlC','on', 'SimIntegrity','on', 'SimParseCustomCode','on', ...
    'SimulationMode','normal');
try, set_param(name,'FastRestart','off'); catch, end

%% ---------------- Bloques del nivel raiz ----------------
add_block('simulink/Sources/Clock', m('Clock'), 'Position',[275 55 295 75]);
add_block('simulink/Sinks/Display', m('Confidence'),  'Position',[910 -10 1000 20]);
add_block('simulink/Sinks/Display', m('Confidence1'), 'Position',[850 -65 940 -35]);
add_block('simulink/Sources/Constant', m('Constant'),  'Position',[295 -105 325 -75], ...
    'Value','0', 'OutDataTypeStr','boolean');
add_block('simulink/Sources/Constant', m('Constant1'), 'Position',[345 245 370 265], 'Value','1');
add_block('simulink/Sources/Constant', m('Constant2'), 'Position',[345 275 370 295], 'Value','0');
add_block('simulink/Sources/Constant', m('Constant3'), 'Position',[15 105 40 125], 'Value','1');
add_block('simulink/Sources/Constant', m('Constant4'), 'Position',[15 135 40 155], 'Value','0');
add_block('simulink/Sources/Constant', m('Home'), 'Position',[315 310 430 340], 'Value','[0; 0; 1; 0]');
add_block('simulink/Sources/Constant', m('Position Simulation'), 'Position',[135 310 250 340], ...
    'Value','[0; 0; 1; 0]', 'BlockMirror','on');
add_block('simulink/Sources/Constant', m('Waypoints'), 'Position',[-135 -40 0 70], 'Value', ...
    sprintf(['[ ...\n   -1.5, -1.5, 1.0, 0.0;\n   -0.5, -1.5, 1.0, 0.0;\n    0.5, -1.5, 1.0, 0.0;\n' ...
             '    1.5, -1.5, 1.0, 0.0;\n    1.5,  0.0, 1.0, 0.0;\n    1.5,  1.5, 1.0, 0.0\n]']));
add_block('simulink/Signal Routing/Manual Switch', m('Manual Switch'), 'Position',[70 117 100 153]);
set_param(m('Manual Switch'),'sw','0');  % posicion inferior = Constant4 (0): mision detenida
add_block('simulink/Math Operations/Gain', m('Multiply'), 'Position',[860 -3 885 13], 'Gain','100');
add_block('simulink/Sinks/Display', m('PoseD'),  'Position',[565 269 650 341]);
add_block('simulink/Sinks/Display', m('Status'), 'Position',[630 64 715 136]);
add_block('simulink/Signal Routing/Switch', m('Switch'),  'Position',[500 187 550 353], ...
    'Criteria','u2 > Threshold', 'InputSameDT','off', 'SaturateOnIntegerOverflow','off');
add_block('simulink/Signal Routing/Switch', m('Switch1'), 'Position',[400 247 455 293], ...
    'Criteria','u2 > Threshold', 'InputSameDT','off', 'SaturateOnIntegerOverflow','off');

matlab_fn(m('Trayectory SM'),        [155 120 310 225],  'TrayectorySM_block.m',       '0.05');
matlab_fn(m('Image Batch Capture'),  [380 -32 605 82],   'CaptureFOMOBatch_block.m',   '');
matlab_fn(m('FOMO Inference'),       [665 -76 835 31],   'InferFOMOBatch_block.m',     '');
matlab_fn(m('Fire Waypoints'),       [790 77 860 123],   'FireWaypoints_block.m',      '');
if o.Visualization
    matlab_fn(m('MATLAB Function'),  [715 -282 895 -118], 'VisualizeFOMOBatch_block.m', '');
    for k = 0:6
        tname = 'Terminator'; if k > 0, tname = sprintf('Terminator%d',k); end
        add_block('simulink/Sinks/Terminator', m(tname), 'Position',[910 -283+25*k 925 -267+25*k]);
    end
end

%% ---------------- Subsistema "Video 3D Capture" ----------------
vsub = m('Video 3D Capture');
add_block('built-in/Subsystem', vsub, 'Position',[155 -36 255 61]);
if strcmpi(o.Camera,'webcam')
    cam = [vsub '/From Video Device'];
    try
        add_block('imaqlib/From Video Device', cam, 'Position',[25 119 135 171]);
    catch err
        error('EI:Rebuild', ['No se pudo agregar From Video Device (%s).\nInstala Image Acquisition ' ...
            'Toolbox + "OS Generic Video Interface", o usa rebuild_trayectory_model(''Camera'',''synthetic'').'], err.message);
    end
    try_set(cam, 'OutputPortsMode','One multidimensional signal');
    try_set(cam, 'DataType','uint8');
    try_set(cam, 'ReturnedColorSpace','rgb');
    try_set(cam, 'SampleTime','1/30');
    try_set(cam, 'VideoFormat','MJPG_1280x720');
    fprintf('Camara: "%s", formato "%s". Cambialo con doble clic en %s.\n', ...
        safe_get(cam,'Device'), safe_get(cam,'VideoFormat'), cam);
else
    cam = [vsub '/Synthetic Camera'];
    add_block('simulink/Sources/Constant', cam, 'Position',[25 119 135 171], ...
        'Value','zeros(720,1280,3,''uint8'')', 'OutDataTypeStr','Inherit: Inherit from ''Constant value''', ...
        'SampleTime','1/30');
end
[~, camName] = fileparts(cam);
matlab_fn([vsub '/Video 3D Capture'], [165 90 280 200], 'VideoDeviceToQUARC_block.m', '');
outs = {'RGB',[360 103 390 117]; 'newFrame',[360 158 390 172]; 'framNum',[360 213 390 227]; 'Camera',[360 253 390 267]};
for k = 1:size(outs,1)
    add_block('simulink/Sinks/Out1', [vsub '/' outs{k,1}], 'Position',outs{k,2}, 'Port',num2str(k));
end
link(vsub, [camName '/1'], 'Camera/1');
link(vsub, [camName '/1'], 'Video 3D Capture/1');
link(vsub, 'Video 3D Capture/1', 'RGB/1');
link(vsub, 'Video 3D Capture/2', 'newFrame/1');
link(vsub, 'Video 3D Capture/3', 'framNum/1');

viewer = m('Video Viewer');
try
    add_block('visionsinks/Video Viewer', viewer, 'Position',[285 -215 400 -145]);
catch
    warning('EI:Rebuild','Computer Vision Toolbox no disponible: Video Viewer se reemplaza por un Terminator.');
    add_block('simulink/Sinks/Terminator', viewer, 'Position',[285 -190 300 -170]);
end

%% ---------------- Conexiones (identicas al original) ----------------
link(name, 'Home/1', 'Switch/3');
link(name, 'Switch1/1', 'Switch/2');
link(name, 'Trayectory SM/1', 'Switch/1');
link(name, 'Switch/1', 'PoseD/1');
link(name, 'Manual Switch/1', 'Trayectory SM/1');
link(name, 'Position Simulation/1', 'Trayectory SM/2');
link(name, 'Trayectory SM/3', 'Switch1/2');
link(name, 'Constant1/1', 'Switch1/1');
link(name, 'Constant2/1', 'Switch1/3');
link(name, 'Constant3/1', 'Manual Switch/1');
link(name, 'Constant4/1', 'Manual Switch/2');
link(name, 'Image Batch Capture/3', 'Status/1');
link(name, 'Video 3D Capture/1', 'Image Batch Capture/1');
link(name, 'Video 3D Capture/2', 'Image Batch Capture/3');
link(name, 'Video 3D Capture/3', 'Image Batch Capture/4');
link(name, 'Video 3D Capture/4', 'Video Viewer/1');
link(name, 'Clock/1', 'Image Batch Capture/5');
link(name, 'FOMO Inference/2', 'Multiply/1');
link(name, 'Multiply/1', 'Confidence/1');
link(name, 'FOMO Inference/1', 'Confidence1/1');
link(name, 'Image Batch Capture/1', 'FOMO Inference/1');
link(name, 'Image Batch Capture/2', 'FOMO Inference/2');
link(name, 'Image Batch Capture/2', 'Trayectory SM/3');
link(name, 'Waypoints/1', 'Trayectory SM/4');
if o.Visualization
    link(name, 'Image Batch Capture/1', 'MATLAB Function/1');
    link(name, 'Image Batch Capture/2', 'MATLAB Function/2');
    % mismo orden que el original: out3->Terminator2, out4->Terminator3, out5..7->Terminator4..6
    terms = {'Terminator','Terminator1','Terminator2','Terminator3','Terminator4','Terminator5','Terminator6'};
    for k = 1:7
        link(name, sprintf('MATLAB Function/%d',k), [terms{k} '/1']);
    end
end
if o.ConnectTrigger
    delete_block(m('Constant'));
    add_block('simulink/Discrete/Unit Delay', m('Trigger Delay'), 'Position',[320 -100 350 -80], ...
        'SampleTime','0.05', 'InitialCondition','false');
    link(name, 'Trayectory SM/2', 'Trigger Delay/1');
    link(name, 'Trigger Delay/1', 'Image Batch Capture/2');
    % Al cerrar el lazo Simulink ya no puede inferir tipos/tamanos por propagacion:
    % se declaran explicitamente las salidas que participan en el lazo.
    set_data(m('Trayectory SM'), 'pointDesired', 'double', '[4 1]');
    set_data(m('Trayectory SM'), 'startCamera', 'boolean', '1');
    set_data(m('Trayectory SM'), 'homeToggle', 'boolean', '1');
    set_data(m('Image Batch Capture'), 'ImgCaptured', 'boolean', '1');
    set_data(m('Image Batch Capture'), 'Status', 'int32', '1');
else
    link(name, 'Constant/1', 'Image Batch Capture/2');  % original: Trigger = false constante
end

note = Simulink.Annotation(name, sprintf(['Reconstruido con rebuild_trayectory_model en MATLAB %s (%s).\n' ...
    'Camara: %s | ConnectTrigger: %d | Visualizacion: %d'], version('-release'), ...
    char(datetime('now','Format','yyyy-MM-dd HH:mm')), o.Camera, o.ConnectTrigger, o.Visualization));
note.Position = [-135 380 420 420];

save_system(name, target);
file = target;
fprintf('Modelo guardado: %s\n', target);

if o.SetupNative
    close_system(name, 0);
    preflight_native(root, name);
    fprintf('Enlazando el detector nativo (setup_custom_model)...\n');
    setup_custom_model(name);
    if o.Visualization
        fprintf(['El bloque "MATLAB Function" (visualizacion) necesita ademas:\n' ...
                 '    setup_fomo_visualization\n']);
    end
else
    fprintf('Siguiente paso: setup_custom_model(''%s'')\n', name);
end
end

%% =================== utilidades ===================
function preflight_native(root, name)
% Requisitos para compilar el DLL de Edge Impulse, verificados en R2023a:
%  * MinGW-w64: con Visual C++ el SDK no compila (usa INCBIN, #warning y __attribute__ de GCC).
%  * CMake >= 3.20: R2023a trae CMake 3.19.7 (R2023b trae uno mas nuevo).
cfg = config();
hint = sprintf(['\nEl modelo YA quedo guardado en %s.slx; solo falta compilar el detector.\n' ...
    'Cuando lo corrijas ejecuta: setup_custom_model(''%s'')'], fullfile(root,name), name);
portable = isfile(fullfile(cfg.portableMinGW,'bin','g++.exe'));
c = mex.getCompilerConfigurations('C++','Selected');
if ~portable && (isempty(c) || ~contains(c.Name,'MinGW'))
    current = 'ninguno'; if ~isempty(c), current = c.Name; end
    error('EI:RebuildCompiler', ['El detector solo compila con MinGW-w64 (compilador actual: %s).\n' ...
        '1) Home > Add-Ons > "MATLAB Support for MinGW-w64 C/C++/Fortran Compiler".\n' ...
        '2) mex -setup C++   y   mex -setup C   (elige MinGW64).%s'], current, hint);
end
cmake = cfg.cmake;
if isempty(cmake)
    [s,~] = system('cmake --version');
    if s == 0, cmake = 'cmake'; else, cmake = fullfile(matlabroot,'bin','win64','cmake','bin','cmake.exe'); end
end
[s,out] = system(['"' cmake '" --version']);
tok = regexp(out,'version (\d+)\.(\d+)','tokens','once');
if s ~= 0 || isempty(tok) || str2double(tok{1})*100 + str2double(tok{2}) < 320
    vs = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe';
    suggestion = 'instala CMake >= 3.20 (cmake.org) y agregalo al PATH';
    if isfile(vs), suggestion = sprintf('en config.m pon:\n   cfg.cmake = ''%s'';', vs); end
    found = 'no encontrado'; if ~isempty(tok), found = sprintf('%s.%s', tok{1}, tok{2}); end
    error('EI:RebuildCMake', 'Se necesita CMake >= 3.20 (encontrado: %s en %s).\nSolucion: %s%s', ...
        found, cmake, suggestion, hint);
end
end

function matlab_fn(path, position, scriptFile, sampleTime)
add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', position);
chart = find(sfroot, '-isa','Stateflow.EMChart', 'Path', path);
assert(numel(chart) == 1, 'EI:Rebuild', 'No se encontro el chart de %s', path);
root = fileparts(mfilename('fullpath'));
chart.Script = fileread(fullfile(root,'simulink','trayectory_blocks',scriptFile));
if ~isempty(sampleTime)
    set_param(path, 'SystemSampleTime', sampleTime);  % igual que en el .slx original
end
end

function set_data(blockPath, dataName, dataType, dataSize)
chart = find(sfroot, '-isa','Stateflow.EMChart', 'Path', blockPath);
d = chart.find('-isa','Stateflow.Data', 'Name', dataName);
assert(numel(d) == 1, 'EI:Rebuild', 'No se encontro %s en %s', dataName, blockPath);
d.DataType = dataType;
d.Props.Array.Size = dataSize;
end

function link(sys, from, to)
add_line(sys, from, to, 'autorouting','on');
end

function try_set(block, param, value)
try
    set_param(block, param, value);
catch err
    warning('EI:Rebuild','No se pudo fijar %s=%s en %s: %s', param, value, block, err.message);
end
end

function v = safe_get(block, param)
try, v = get_param(block, param); catch, v = '?'; end
end
