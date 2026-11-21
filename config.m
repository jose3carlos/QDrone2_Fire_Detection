function cfg = config()
% Central project configuration. Change ONLY MODEL_PATH to switch detectors.
MODEL_PATH = "models/fire_hazard-cpp-mcu-v7-impulse-#1.zip";
root = fileparts(mfilename('fullpath'));
cfg.root = root;
cfg.modelPath = fullfile(root, MODEL_PATH);
cfg.buildRoot = fullfile(root, 'build');
cfg.minConfidence = 0.50; % fraction; SDK also has its own detection floor
% Exactly one foreground class; use detected for presence/absence.
% objectClass is retained as false for compatibility with existing Simulink ports.
cfg.maxDetections = 4096; % includes SDK zero-confidence padding
cfg.timeoutMs = 0; % synchronous deadline check AFTER completion, never unsafe cancel
cfg.debugLog = false;
cfg.image.resize = 'model'; % follow exported resize geometry, base MATLAB only
cfg.image.sampleTime = 0.1; % simulation grid, NOT an inference period
cfg.image.channels = 'model'; % inferred from exported RGB/Grayscale DSP config
cfg.cmake = ''; % empty: PATH, then the CMake bundled with MATLAB
% R2023a and older bundle CMake < 3.20 (R2023a: 3.19.7), which cannot build this project.
% Use the CMake shipped with Visual Studio 2022 Community when it is installed.
vsCMake = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe';
if isMATLABReleaseOlderThan('R2023b') && isfile(vsCMake), cfg.cmake = vsCMake; end
cfg.generator = ''; % empty: selected MEX compiler decides VS2022/MinGW Makefiles
cfg.parallelJobs = 4;
cfg.portableMinGW = fullfile(root, 'tools', 'mingw64');
end

