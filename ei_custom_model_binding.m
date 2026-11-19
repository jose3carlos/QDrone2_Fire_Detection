function ei_custom_model_binding(action, model)
% Managed custom-model integration. Does not save models or change timing.
setup_project(false);
model = get_param(model,'Name');
switch action
    case 'install'
        for name = {'PostLoadFcn','InitFcn'}
            original = unmanaged(get_param(model,name{1}),'matlab');
            if strcmp(name{1},'PostLoadFcn'), phase='load'; else, phase='refresh'; end
            set_param(model,name{1},[bootstrap(phase) original]);
        end
    case 'load'
        % Loading is stopped: a moved project may safely select/build its own DLL.
        setup_project(); % Configure this computer's compiler before rebuilding.
    case 'refresh'
        % InitFcn runs before target compilation. Never build/start a simulation here.
    otherwise
        error('EI:CustomBinding','Unknown custom model action: %s',action);
end
assert(strcmp(get_param(model,'SimulationMode'),'normal'),'EI:SimulationMode', ...
    'The custom native integration supports Windows normal simulation.');
assert(strcmp(get_param(model,'FastRestart'),'off'),'EI:FastRestart', ...
    'Disable Fast Restart for the native engine lifecycle; other model settings are preserved.');
cfg=config(); manifest=fullfile(cfg.buildRoot,'active_model.mat');
assert(isfile(manifest),'EI:CustomBuild','Run setup_custom_model before simulating.');
data=load(manifest,'artifact'); a=data.artifact;
assert(isfield(a,'projectRoot') && strcmpi(a.projectRoot,cfg.root) && ...
    strcmp(a.matlabVersion,version) && strcmp(a.host,getenv('COMPUTERNAME')) && ...
    strcmp(a.modelPath,char(cfg.modelPath)) && isfile(a.dll) && isfile(a.importLibrary) && ...
    isfile(fullfile(a.includeDir,'ei_wrapper.h')), ...
    'EI:CustomBuild','Native build belongs to another location/environment. Run setup_custom_model.');
compiler=mex.getCompilerConfigurations(get_param(model,'SimTargetLang'),'Selected');
assert(~isempty(compiler) && (contains(compiler.Name,'MinGW')==contains(a.compiler,'MinGW')), ...
    'EI:CompilerMismatch','Simulation and native build compiler families differ. Run setup_custom_model.');
if isfile(cfg.modelPath)
    assert(strcmp(a.modelHash,ei_hash(cfg.modelPath,true)),'EI:ModelChanged', ...
        'The ZIP changed. Run setup_custom_model to build and bind the new model.');
end
workspace=get_param(model,'ModelWorkspace');
assert(strcmp(workspace.DataSource,'Model File'),'EI:ModelWorkspace', ...
    'Custom integration metadata requires a model-file workspace; external workspaces are not modified.');
previous=struct('includeDir','','importLibrary','');
if hasVariable(workspace,'eiCustomBinding')
    previous=getVariable(workspace,'eiCustomBinding');
    assert(isstruct(previous) && isfield(previous,'includeDir') && isfield(previous,'importLibrary'), ...
        'EI:BindingMetadata','The model variable eiCustomBinding is reserved for native bindings.');
    if isfield(previous,'imageContract')
        assert(isequal(previous.imageContract,[a.metadata.height a.metadata.width a.metadata.channels]), ...
            'EI:ImageContractChanged', ...
            'The replacement detector changed image dimensions/channels. Existing block inputs are preserved; use a compatible export.');
    end
end
set_if_changed(model,'SimUserIncludeDirs',merge_path(get_param(model,'SimUserIncludeDirs'),previous.includeDir,a.includeDir));
set_if_changed(model,'SimUserLibraries',merge_path(get_param(model,'SimUserLibraries'),previous.importLibrary,a.importLibrary));
header=sprintf('#include "ei_wrapper.h"\n#include "mex.h"');
initializer=sprintf(['{ int32_t ei_custom_status=ei_sim_start(%.9ef,%du,%du,%du,%.17g);\n' ...
    'if(ei_custom_status<0) { char ei_custom_error[1024]; ei_get_last_error(ei_custom_error,1024);\n' ...
    'mexErrMsgIdAndTxt("EI:SimInit","%%s",ei_custom_error); } }'], ...
    cfg.minConfidence,a.falseIndex,cfg.maxDetections,cfg.debugLog,cfg.timeoutMs);
merge_code(model,'SimCustomHeaderCode',header);
merge_code(model,'SimCustomInitializer',initializer);
merge_code(model,'SimCustomTerminator','ei_sim_stop();');
binding=struct('includeDir',a.includeDir,'importLibrary',a.importLibrary,'buildKey',a.key, ...
    'imageContract',[a.metadata.height a.metadata.width a.metadata.channels]);
if ~hasVariable(workspace,'eiCustomBinding') || ~isequal(getVariable(workspace,'eiCustomBinding'),binding)
    assignin(workspace,'eiCustomBinding',binding);
end
% Release the standalone gateway before the simulation acquires this DLL.
close_ei_model();
dllFolder=fileparts(a.dll);
paths=strsplit(getenv('PATH'),pathsep);
if ~any(strcmpi(paths,dllFolder)), setenv('PATH',[dllFolder pathsep getenv('PATH')]); end
end

function result=merge_path(original,previous,current)
result=original;
if ~isempty(previous)
    % Only remove the exact path owned by this integration, including quoting.
    expression=['(?<!\S)"' regexptranslate('escape',previous) '"(?=\s|$)'];
    result=regexprep(result,expression,'');
end
entry=['"' current '"'];
if ~contains(result,entry), result=[result newline entry]; end
result=strtrim(result);
end

function merge_code(model,parameter,code)
original=unmanaged(get_param(model,parameter),'c');
managed=sprintf('/* EICUSTOM_BINDING_BEGIN */\n%s\n/* EICUSTOM_BINDING_END */\n',code);
set_if_changed(model,parameter,[managed original]);
end

function result=unmanaged(original,language)
if strcmp(language,'c')
    pattern='/\* EICUSTOM_BINDING_BEGIN \*/.*?/\* EICUSTOM_BINDING_END \*/\r?\n?';
else
    pattern='% EICUSTOM_BINDING_BEGIN.*?% EICUSTOM_BINDING_END\r?\n?';
end
result=regexprep(original,pattern,'','dotall');
end

function set_if_changed(model,parameter,value)
if ~strcmp(get_param(model,parameter),value), set_param(model,parameter,value); end
end

function code=bootstrap(phase)
% No absolute paths: the callback finds its project before calling any helper.
code=sprintf(['%% EICUSTOM_BINDING_BEGIN\n' ...
    'eiCustomModelName=bdroot;\n' ...
    'eiCustomProjectRoot=fileparts(get_param(eiCustomModelName,''FileName''));\n' ...
    'while ~isfile(fullfile(eiCustomProjectRoot,''setup_custom_model.m''))\n' ...
    '    eiCustomParent=fileparts(eiCustomProjectRoot);\n' ...
    '    assert(~strcmp(eiCustomParent,eiCustomProjectRoot) && ~isempty(eiCustomParent),''EI:ProjectMissing'',''Keep this model inside the Edge Impulse project folder.'');\n' ...
    '    eiCustomProjectRoot=eiCustomParent;\n' ...
    'end\n' ...
    'addpath(eiCustomProjectRoot,''-begin'');\n' ...
    'ei_custom_model_binding(''%s'',eiCustomModelName);\n' ...
    'clear eiCustomModelName eiCustomProjectRoot eiCustomParent\n' ...
    '%% EICUSTOM_BINDING_END\n'],phase);
end
