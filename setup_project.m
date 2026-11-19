function artifact = setup_project(buildNative)
% SETUP_PROJECT Configure session paths, build the DLL and verify MATLAB loading.
if nargin < 1, buildNative = true; end
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root,'matlab'),fullfile(root,'simulink'),fullfile(root,'scripts'));
cfg = config();
assert(ispc && strcmp(computer('arch'),'win64'),'EI:Platform','Windows x64 is required.');
portable=isfile(fullfile(cfg.portableMinGW,'bin','g++.exe'));
if portable
    setenv('MW_MINGW64_LOC',cfg.portableMinGW);
    setenv('PATH',[fullfile(cfg.portableMinGW,'bin') pathsep getenv('PATH')]);
end
artifact = [];
if buildNative
    if portable
        assert(~contains(cfg.portableMinGW,' '),'EI:CompilerPath', ...
            'The bundled MinGW compiler requires a path without spaces. Copy the project to a writable folder such as C:\EI_FOMO.');
    end
    languages={'C++','C'};options={'mingw64_g++.xml','mingw64.xml'};
    for k=1:numel(languages)
        selected=mex.getCompilerConfigurations(languages{k},'Selected');
        if portable && (isempty(selected) || ~contains(selected.Name,'MinGW') || ~strcmpi(selected.Location,cfg.portableMinGW))
            optionFile=fullfile(matlabroot,'bin','win64','mexopts',options{k});
            assert(isfile(optionFile),'EI:Compiler','Install a compiler supported by your MATLAB release.');
            mex(['-setup:' optionFile],languages{k});
        elseif isempty(selected)
            mex('-setup',languages{k});
        end
    end
    artifact = build_model();
    load_ei_model();
    disp(get_ei_model_info());
    close_ei_model();
end
end
