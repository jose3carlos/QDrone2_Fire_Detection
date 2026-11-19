function artifact = setup_custom_model(model)
% SETUP_CUSTOM_MODEL Attach the native backend without changing model timing.
% Run in the MATLAB Command Window after copying the project or changing ZIP.
% Existing blocks, solver settings and unrelated custom code are preserved.
if nargin < 1, model = 'Trayectory_Model'; end
setup_project(false); cfg = config(); ei_assert_stopped();
[~, name] = fileparts(char(model));
if bdIsLoaded(name)
    assert(strcmp(get_param(name,'Dirty'),'off'),'EI:UnsavedModel', ...
        'Save your existing changes before running setup_custom_model.');
    file = get_param(name,'FileName');
else
    file = which(char(model));
    if isempty(file), file = which([name '.slx']); end
    if isempty(file) && isfile(model), file = char(model); end
end
assert(~isempty(file) && isfile(file),'EI:CustomModel','Cannot locate model %s.',char(model));
file = char(java.io.File(file).getCanonicalPath());
project = char(java.io.File(cfg.root).getCanonicalPath());
assert(startsWith(lower(file),[lower(project) filesep]),'EI:CustomModelLocation', ...
    'Keep the custom model inside the copied project folder (subfolders are supported).');
backupFolder = fullfile(cfg.root,'backups','custom_models');
if ~isfolder(backupFolder), mkdir(backupFolder); end
backupFolder = tempname(backupFolder); mkdir(backupFolder);
[~,~,extension] = fileparts(file);
copyfile(file,fullfile(backupFolder,[name extension]));
fprintf('Custom model backup: %s\n',backupFolder);
artifact = setup_project(); % Reuses the valid local build; never regenerates a model.
load_system(file);
ei_custom_model_binding('install',name);
save_system(name);
fprintf('Native integration ready: %s (build %s). Timing and blocks preserved.\n',name,artifact.key);
end
