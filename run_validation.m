function results = run_validation()
setup_project();cfg=config();
addpath(fullfile(cfg.root,'tests','simulink_tests'));
diaryFile=fullfile(cfg.buildRoot,'matlab_simulink_validation.log');diary(diaryFile);
cleanup=onCleanup(@()diary('off')); %#ok<NASGU>
results=runtests({fullfile(cfg.root,'tests','matlab_tests'),fullfile(cfg.root,'tests','simulink_tests')});
disp(results);save(fullfile(cfg.buildRoot,'validation_results.mat'),'results');
assert(all([results.Passed]),'EI:ValidationFailed','One or more integration tests did not pass. See test report.');
end
