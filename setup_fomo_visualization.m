function visualization=setup_fomo_visualization()
% Optional add-on: compile a gateway without changing native/model sources.
% Run in the MATLAB Command Window while all simulations are stopped.
native=setup_project(); cfg=config();
assert(native.metadata.height==180 && native.metadata.width==180 && native.metadata.channels==1, ...
    'EI:VisualizationShape','This optional batch viewer expects the unchanged 180-by-180 grayscale contract.');
source=fullfile(cfg.root,'visualization','ei_visualization_mex.cpp');
compiler=mex.getCompilerConfigurations('C++','Selected');
identity=ei_hash([ei_hash(source,true) ei_hash(fullfile(cfg.root,'native','ei_wrapper.h'),true) ...
    native.key cfg.root matlabroot version computer('arch') getenv('COMPUTERNAME') ...
    compiler.Name compiler.Version compiler.Location]);
key=identity(1:20); folder=fullfile(cfg.buildRoot,'visualization',key);
gateway=['ei_visual_gateway_' key];
if ~isfolder(folder),mkdir(folder);end
binary=fullfile(folder,[gateway '.' mexext]);
if ~isfile(binary)
    mex('-R2018a',['-I' fullfile(cfg.root,'native')],source, ...
        '-outdir',folder,'-output',gateway);
end
addpath(folder);
metadata=feval(gateway,'info',native.dll);
assert(isequal([metadata.height metadata.width metadata.channels],[180 180 1]), ...
    'EI:VisualizationShape','Compiled detector image contract does not match.');
feval(gateway,'unload');
visualization=struct('key',key,'nativeKey',native.key,'projectRoot',cfg.root, ...
    'gateway',gateway,'folder',folder,'binary',binary,'matlabVersion',version, ...
    'host',getenv('COMPUTERNAME'));
save(fullfile(cfg.buildRoot,'visualization','active_visualization.mat'),'visualization');
fprintf('Optional FOMO visualization ready (native build %s). Existing model unchanged.\n',native.key);
end
