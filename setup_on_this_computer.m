function artifact=setup_on_this_computer()
% Run once after copying the project, in the MATLAB Command Window.
% Build locally and regenerate the standard example's machine-specific links.
setup_project(false);cfg=config();ei_assert_stopped();
example='EdgeImpulse_YOLO_Simulink';
if license('test','Simulink') && bdIsLoaded(example)
    assert(strcmp(get_param(example,'Dirty'),'off'),'EI:UnsavedModel', ...
        'Save your open example before regenerating its native settings.');
end
artifact=setup_project();
if license('test','Simulink')
    file=fullfile(cfg.root,'simulink',[example '.slx']);
    if isfile(file)
        backups=fullfile(cfg.root,'simulink','transfer_backups');
        if ~isfolder(backups),mkdir(backups);end
        copyfile(file,[tempname(backups) '.slx']);
    end
    create_simulink_model();
end
fprintf('Project ready at %s\n',cfg.root);
end
