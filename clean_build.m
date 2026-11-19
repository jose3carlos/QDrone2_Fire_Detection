function clean_build()
% Remove generated cache only, after releasing all engine consumers.
root=fileparts(mfilename('fullpath')); setup_project(false);
ei_assert_stopped(); close_ei_model();
assert(~bdIsLoaded('EdgeImpulse_YOLO_Simulink'),'EI:ModelLoaded', ...
    'Save and close EdgeImpulse_YOLO_Simulink before cleaning its native libraries.');
clear mex;
cfg=config(); target=char(java.io.File(cfg.buildRoot).getCanonicalPath());
expected=char(java.io.File(fullfile(root,'build')).getCanonicalPath());
assert(strcmpi(target,expected),'EI:CleanPath','Refusing to delete outside the project build folder.');
if isfolder(target), rmdir(target,'s'); end
end
