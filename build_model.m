function artifact = build_model(force)
% Build each model + source + compiler fingerprint once. Never modifies the ZIP.
if nargin<1, force=false; end
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'scripts'),fullfile(root,'matlab'));
cfg=config();
ei_assert_stopped(); close_ei_model();
assert(isfile(cfg.modelPath) || isfolder(cfg.modelPath),'EI:ModelPath','Model does not exist: %s',cfg.modelPath);
compiler=mex.getCompilerConfigurations('C++','Selected');
assert(~isempty(compiler),'EI:Compiler','No selected C++ compiler. Run setup_project or mex -setup C++.');
if isempty(cfg.cmake)
    [s,~]=system('cmake --version');
    if s==0, cmake='cmake'; else, cmake=fullfile(matlabroot,'bin','win64','cmake','bin','cmake.exe'); end
else, cmake=cfg.cmake;
end
assert(isfile(cmake) || strcmp(cmake,'cmake'),'EI:CMake','Install CMake >=3.20 or set cfg.cmake.');
if isfile(cfg.modelPath), modelHash=ei_hash(cfg.modelPath,true);
else, modelHash=hash_tree(cfg.modelPath); end
nativeHash=ei_hash([hash_tree(fullfile(root,'native')) hash_tree(fullfile(root,'tests','native_tests'))]);
builderHash=ei_hash([fileread(mfilename('fullpath')+".m") fileread(fullfile(root,'scripts','ei_patch_sdk.m')) ...
    fileread(fullfile(root,'scripts','inspect_model.m'))]);
% CMake caches, MEX binaries and saved absolute paths belong to this location,
% MATLAB installation and host. A copied build must never reuse the old paths.
environment=strjoin({root,matlabroot,version,computer('arch'),getenv('COMPUTERNAME')},'|');
identity=ei_hash([modelHash nativeHash builderHash compiler.Name compiler.Version compiler.Location cfg.generator environment]);
key=identity(1:20); work=fullfile(cfg.buildRoot,key); folder=fullfile(work,'export');
gateway=['ei_gateway_' key];
artifact=struct('key',key,'modelHash',modelHash,'modelPath',char(cfg.modelPath),'folder',work, ...
    'dll',fullfile(work,'cmake','bin',['ei_inference_' key '.dll']),'gateway',gateway, ...
    'gatewayPath',fullfile(work,'matlab'),'includeDir',fullfile(root,'native'), ...
    'importLibrary','','compiler',compiler.Name,'projectRoot',root, ...
    'matlabVersion',version,'host',getenv('COMPUTERNAME'));
manifest=fullfile(work,'artifact.mat');
if ~force && isfile(manifest)
    loaded=load(manifest,'artifact'); artifact=loaded.artifact;
    local=isfield(artifact,'projectRoot') && strcmpi(artifact.projectRoot,root) && strcmpi(artifact.folder,work);
    if local && isfile(artifact.dll) && isfile(artifact.importLibrary) && isfile(fullfile(artifact.gatewayPath,[gateway '.' mexext]))
        artifact=resolve_mapping(cfg,artifact);
        save_active(cfg,artifact); fprintf('Using cached native build %s\n',key); return;
    end
end
if ~isfolder(folder)
    mkdir(folder);
    if isfile(cfg.modelPath)
        validate_zip(cfg.modelPath); unzip(cfg.modelPath,folder);
    else, copyfile(fullfile(cfg.modelPath,'*'),folder); end
end
metadata=dir(fullfile(folder,'**','model_metadata.h'));
if ~isfile(fullfile(folder,'model-parameters','model_metadata.h'))
    assert(numel(metadata)==1,'EI:ExportRoot','Cannot uniquely locate model-parameters/model_metadata.h.');
    folder=fileparts(fileparts(fullfile(metadata(1).folder,metadata(1).name)));
end
artifact.metadata=inspect_model(folder);
ei_patch_sdk(folder);
artifact=resolve_mapping(cfg,artifact);
buildDir=fullfile(work,'cmake');
generator=cfg.generator; extra='';
if contains(compiler.Name,'MinGW')
    toolRoot=getenv('MW_MINGW64_LOC'); if isempty(toolRoot), toolRoot=compiler.Location; end
    setenv('PATH',[fullfile(toolRoot,'bin') pathsep getenv('PATH')]);
    if isempty(generator), generator='MinGW Makefiles'; end
    extra=sprintf(' -DCMAKE_MAKE_PROGRAM="%s" -DCMAKE_C_COMPILER="%s" -DCMAKE_CXX_COMPILER="%s"', ...
        unixpath(fullfile(toolRoot,'bin','mingw32-make.exe')),unixpath(fullfile(toolRoot,'bin','gcc.exe')),unixpath(fullfile(toolRoot,'bin','g++.exe')));
    artifact.importLibrary=fullfile(buildDir,'lib',['libei_inference_' key '.dll.a']);
    artifact.fixtureLibrary=fullfile(buildDir,'lib','libei_contract_fixture.dll.a');
else
    if isempty(generator), generator='Visual Studio 17 2022'; end
    extra=' -A x64'; artifact.importLibrary=fullfile(buildDir,'lib',['ei_inference_' key '.lib']);
    artifact.fixtureLibrary=fullfile(buildDir,'lib','ei_contract_fixture.lib');
end
run(sprintf('"%s" -S "%s" -B "%s" -G "%s" -DEI_MODEL_DIR="%s" -DEI_BINARY_NAME=ei_inference_%s -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON%s', ...
    cmake,fullfile(root,'native'),buildDir,generator,unixpath(folder),key,extra));
run(sprintf('"%s" --build "%s" --config Release --parallel %d',cmake,buildDir,cfg.parallelJobs));
if contains(compiler.Name,'MinGW')
    dll=fullfile(toolRoot,'bin','libwinpthread-1.dll');
    if isfile(dll), copyfile(dll,fileparts(artifact.dll)); end
end
ctest=fullfile(fileparts(cmake),'ctest.exe');
if isfile(ctest), run(sprintf('"%s" --test-dir "%s" -C Release --output-on-failure',ctest,buildDir));
else
    run(sprintf('"%s"',fullfile(buildDir,'native_unit_tests.exe')));
    run(sprintf('"%s"',fullfile(buildDir,'bin','native_integration_tests.exe')));
end
if ~isfolder(artifact.gatewayPath), mkdir(artifact.gatewayPath); end
mex('-R2018a',['-I' fullfile(root,'native')],fullfile(root,'native','ei_matlab_mex.cpp'), ...
    '-outdir',artifact.gatewayPath,'-output',gateway);
addpath(artifact.gatewayPath); setenv('PATH',[fileparts(artifact.dll) pathsep getenv('PATH')]);
actual=feval(gateway,'info',artifact.dll);
assert(actual.width==artifact.metadata.width && actual.height==artifact.metadata.height && actual.channels==artifact.metadata.channels && ...
    isequal(actual.labels,artifact.metadata.labels),'EI:Metadata','Compiled model metadata mismatch.');
feval(gateway,'unload');
artifact.builtAt=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
save(manifest,'artifact'); save_active(cfg,artifact);
fprintf('Built and verified DLL + MATLAB gateway: %s\n',artifact.dll);
end
function run(command)
[s,out]=system(command); fprintf('%s',out); assert(s==0,'EI:Build','Command failed (%d): %s',s,command);
end
function text=unixpath(text), text=strrep(char(text),'\','/'); end
function save_active(cfg,artifact)
if ~isfolder(cfg.buildRoot), mkdir(cfg.buildRoot); end
save(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');
end
function artifact=resolve_mapping(cfg,artifact)
assert(artifact.metadata.classCount==1 && numel(artifact.metadata.labels)==1, ...
    'EI:Labels','Exactly one foreground class is required.');
artifact.falseIndex=0;
artifact.booleanLabels=artifact.metadata.labels; % legacy metadata field: sole class
end
function h=hash_tree(root)
files=dir(fullfile(root,'**','*')); files=files(~[files.isdir]);
[~,ix]=sort({files.name}); files=files(ix); parts=cell(1,numel(files));
for i=1:numel(files)
    p=fullfile(files(i).folder,files(i).name);
    parts{i}=[p(numel(char(root))+1:end) ei_hash(p,true)];
end
h=ei_hash(strjoin(parts,'|'));
end
function validate_zip(path)
z=java.util.zip.ZipFile(java.io.File(char(path))); cleaner=onCleanup(@()z.close()); %#ok<NASGU>
entries=z.entries();
while entries.hasMoreElements()
    e=entries.nextElement(); name=strrep(char(e.getName()),'\','/');
    assert(~startsWith(name,'/') && ~contains(name,':') && ...
        ~any(strcmp(strsplit(name,'/'),'..')),'EI:ArchivePath','Unsafe archive entry: %s',name);
end
end
