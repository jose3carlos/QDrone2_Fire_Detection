function enable_live_source(model)
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'EI:Running','Stop simulation before changing source.');
cfg=config(); data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');a=data.artifact;
delete_line(model,'Image/1','Event Detector/1');
delete_line(model,'New Image Event/1','Event Detector/2');
delete_block([model '/Image']); delete_block([model '/New Image Event']);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[model '/Live Image Mailbox'], ...
    'FunctionName','ei_live_source','Parameters',sprintf('%d,%d,%.17g,%d',a.metadata.height,a.metadata.width,cfg.image.sampleTime,a.metadata.channels), ...
    'Position',[35 80 190 175]);
add_line(model,'Live Image Mailbox/1','Event Detector/1');
add_line(model,'Live Image Mailbox/2','Event Detector/2');
ei_live_mailbox('reset');set_param(model,'StopTime','inf');
end
