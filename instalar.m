function instalar(varargin)
% INSTALAR  Deja el proyecto listo en esta PC con un solo comando.
%
%   instalar                        % todo: compilador, CMake, detector y Trayectory_Model
%   instalar('Modelo',false)        % solo el detector (sin Simulink)
%   instalar('Visualizacion',false) % sin el bloque de figuras FOMO
%   instalar('DescargarCMake',true) % usa un CMake portable en tools\cmake aunque haya otro
%
% Pasos:
%   1. Revisa la PC y la carpeta (Windows x64, ruta sin espacios).
%   2. Selecciona MinGW-w64 como compilador de mex (el SDK de Edge Impulse no
%      compila con Visual C++).
%   3. Busca un CMake >= 3.20 (PATH, Visual Studio, C:\Program Files\CMake,
%      tools\cmake). Si no hay ninguno, descarga CMake 3.27.9 oficial de Kitware
%      (44 MB) a tools\cmake y verifica su SHA-256.
%   4. Compila el detector (setup_project) y lo prueba con una imagen negra.
%   5. Enlaza Trayectory_Model en esta PC (lo reconstruye si fue guardado con una
%      version de MATLAB mas nueva que la tuya) y prepara la visualizacion.
%
% Lo unico manual: el add-on "MATLAB Support for MinGW-w64 C/C++/Fortran Compiler"
% (Home > Add-Ons). Si falta, instalar se detiene y explica como instalarlo.
% Se puede volver a correr cuantas veces quieras: lo que ya esta listo se reutiliza.

p = inputParser;
addParameter(p, 'Modelo', true, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'Visualizacion', true, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'DescargarCMake', false, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
o = p.Results;
root = fileparts(mfilename('fullpath'));
t0 = tic;

%% 1. PC y carpeta
paso(1, 'Revisando la PC y la carpeta del proyecto');
assert(ispc && strcmp(computer('arch'), 'win64'), 'EI:Instalar', 'Se requiere Windows de 64 bits.');
fprintf('   MATLAB R%s en %s\n', version('-release'), root);
assert(~contains(root, ' '), 'EI:InstalarRuta', ['La carpeta tiene espacios:\n   %s\n' ...
    'Muevela a una ruta corta sin espacios, por ejemplo C:\\QDrone2_Fire_Detection, y vuelve a correr instalar.'], root);
if numel(root) > 60
    warning('EI:InstalarRuta', ['La ruta del proyecto es larga (%d caracteres). Si la compilacion falla por ' ...
        '"path too long", mueve el proyecto a algo como C:\\QDrone2_Fire_Detection.'], numel(root));
end
if o.Modelo && ~license('test', 'Simulink')
    warning('EI:Instalar', 'No hay licencia de Simulink: se prepara solo el detector.');
    o.Modelo = false;
end
setup_project(false);   % rutas de MATLAB del proyecto (no compila nada)

%% 2. Compilador
paso(2, 'Compilador MinGW-w64');
seleccionar_mingw(root);

%% 3. CMake
paso(3, 'CMake 3.20 o mas nuevo');
cmake = asegurar_cmake(root, o.DescargarCMake);
fprintf('   Usando: %s (%s)\n', cmake, version_texto(cmake));

%% 4. Detector
paso(4, 'Compilando el detector (la primera vez tarda de 1 a 5 minutos)');
setup_project;
r = run_ei_inference(zeros(180, 180, 'uint8'));
close_ei_model;
assert(r.status == 2 && ~r.detected, 'EI:Instalar', ...
    'La prueba con imagen negra dio status=%d (se esperaba 2).', r.status);
fprintf('   Prueba con imagen negra: status=2, sin deteccion. Correcto.\n');

%% 5. Modelo Simulink
if o.Modelo
    paso(5, 'Enlazando Trayectory_Model');
    file = fullfile(root, 'Trayectory_Model.slx');
    if bdIsLoaded('Trayectory_Model'), close_system('Trayectory_Model', 0); end
    rebuild = ~isfile(file);
    if ~rebuild
        info = Simulink.MDLInfo(file);
        rebuild = isMATLABReleaseOlderThan(info.ReleaseName);
        if rebuild
            fprintf('   El modelo esta guardado en %s y tu MATLAB es R%s: se reconstruye.\n', ...
                info.ReleaseName, version('-release'));
        end
    end
    if rebuild
        rebuild_trayectory_model('ConnectTrigger', true, 'SetupNative', false);
    end
    setup_custom_model('Trayectory_Model');
    if o.Visualizacion
        setup_fomo_visualization;
    end
end

fprintf('\n==== Listo en %.0f s ====\n', toc(t0));
fprintf('Detector:   r = run_ei_inference(''C:\\ruta\\foto.jpg'')\n');
if o.Modelo
    fprintf('Simulink:   open_system(''Trayectory_Model'')\n');
    fprintf('            (elige tu camara con doble clic en Video 3D Capture > From Video Device)\n');
end
fprintf('Web:        doble clic en web\\iniciar_web.bat (no necesita MATLAB)\n');
end

%% ======================================================================
function paso(n, texto)
fprintf('\n[%d/5] %s\n', n, texto);
end

function seleccionar_mingw(root)
if isfile(fullfile(root, 'tools', 'mingw64', 'bin', 'g++.exe'))
    fprintf('   Usando el MinGW portable de tools\\mingw64 (setup_project lo selecciona).\n');
    return
end
langs = {'C++', 'C'};
for k = 1:numel(langs)
    sel = mex.getCompilerConfigurations(langs{k}, 'Selected');
    if ~isempty(sel) && contains(sel.Name, 'MinGW')
        fprintf('   %s: %s (ya seleccionado)\n', langs{k}, sel.Name);
        continue
    end
    inst = mex.getCompilerConfigurations(langs{k}, 'Installed');
    mingw = [];
    for j = 1:numel(inst)
        if contains(inst(j).Name, 'MinGW'), mingw = inst(j); break; end
    end
    if isempty(mingw)
        error('EI:InstalarMinGW', ['Falta el compilador MinGW-w64 (con Visual C++ el detector no compila).\n' ...
            '  1) En MATLAB: Home > Add-Ons > Get Add-Ons, busca "MinGW" e instala\n' ...
            '     "MATLAB Support for MinGW-w64 C/C++/Fortran Compiler" (pide tu cuenta MathWorks).\n' ...
            '  2) Vuelve a ejecutar:  instalar']);
    end
    antes = 'ninguno';
    if ~isempty(sel), antes = sel.Name; end
    mex(['-setup:' mingw.MexOpt], langs{k});
    fprintf('   %s: seleccionado %s (antes: %s)\n', langs{k}, mingw.Name, antes);
end
end

function cmake = asegurar_cmake(root, soloPortable)
saved = fullfile(root, 'tools', 'cmake_path.txt');
candidatos = {};
if soloPortable
    portable = dir(fullfile(root, 'tools', 'cmake', '*', 'bin', 'cmake.exe'));
    if isempty(portable)
        cmake = descargar_cmake(root);
    else
        cmake = fullfile(portable(1).folder, portable(1).name);
    end
    guardar_cmake(saved, cmake);
    return
end
cfg = config();
if ~isempty(cfg.cmake), candidatos{end+1} = cfg.cmake; end
[s, out] = system('where cmake');
if s == 0, candidatos = [candidatos, strtrim(strsplit(strtrim(out), newline))]; end
vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe';
if isfile(vswhere)
    [s, out] = system(['"' vswhere '" -all -products * -property installationPath']);
    if s == 0
        for d = strtrim(strsplit(strtrim(out), newline))
            if ~isempty(d{1})
                candidatos{end+1} = fullfile(d{1}, 'Common7', 'IDE', 'CommonExtensions', ...
                    'Microsoft', 'CMake', 'CMake', 'bin', 'cmake.exe'); %#ok<AGROW>
            end
        end
    end
end
candidatos{end+1} = 'C:\Program Files\CMake\bin\cmake.exe';
portable = dir(fullfile(root, 'tools', 'cmake', '*', 'bin', 'cmake.exe'));
for k = 1:numel(portable)
    candidatos{end+1} = fullfile(portable(k).folder, portable(k).name); %#ok<AGROW>
end
candidatos{end+1} = fullfile(matlabroot, 'bin', 'win64', 'cmake', 'bin', 'cmake.exe');

cmake = '';
for k = 1:numel(candidatos)
    c = candidatos{k};
    if isfile(c) && version_numero(c) >= 3.20
        cmake = c;
        break
    end
end
if isempty(cmake)
    cmake = descargar_cmake(root);
end
guardar_cmake(saved, cmake);
end

function guardar_cmake(saved, cmake)
% config.m lee este archivo: asi setup_project y build_model usan este CMake.
if ~isfolder(fileparts(saved)), mkdir(fileparts(saved)); end
f = fopen(saved, 'w'); fprintf(f, '%s', cmake); fclose(f);
end

function cmake = descargar_cmake(root)
ver = '3.27.9';
name = ['cmake-' ver '-windows-x86_64'];
url = ['https://github.com/Kitware/CMake/releases/download/v' ver '/' name '.zip'];
sha = 'c14e8b5d1c7be0baf0e7936ce8b5a39c5ee3450b14d7e3b32435083eddd9aff7';
dest = fullfile(root, 'tools', 'cmake');
if ~isfolder(dest), mkdir(dest); end
zipFile = fullfile(dest, [name '.zip']);
fprintf('   No se encontro CMake >= 3.20. Descargando CMake %s oficial de Kitware (44 MB)...\n', ver);
fprintf('   %s\n', url);
websave(zipFile, url, weboptions('Timeout', 300));
got = ei_hash(zipFile, true);
if ~strcmp(got, sha)
    delete(zipFile);
    error('EI:InstalarCMake', 'La descarga de CMake no paso la verificacion SHA-256 (obtenido %s). Intenta de nuevo.', got);
end
unzip(zipFile, dest);
delete(zipFile);
cmake = fullfile(dest, name, 'bin', 'cmake.exe');
assert(isfile(cmake), 'EI:InstalarCMake', 'No se encontro cmake.exe tras descomprimir en %s', dest);
fprintf('   CMake %s verificado e instalado en tools\\cmake.\n', ver);
end

function v = version_numero(cmake)
v = 0;
[s, out] = system(['"' cmake '" --version']);
tok = regexp(out, 'version (\d+)\.(\d+)', 'tokens', 'once');
if s == 0 && ~isempty(tok)
    v = str2double(tok{1}) + str2double(tok{2}) / 100;
end
end

function t = version_texto(cmake)
[~, out] = system(['"' cmake '" --version']);
t = strtrim(regexp(out, 'version [^\r\n]+', 'match', 'once'));
end
