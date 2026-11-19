function output = test_intermitent_images(files)
setup_project(false); model=create_simulink_model();
cfg=config();data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact'); info=data.artifact.metadata;
if nargin<1
    images=repmat({zeros(info.height,info.width,info.channels,'uint8')},1,4);
else
    assert(numel(files)==4,'EI:Images','Supply four image paths.');
    images=cellstr(files); % prepare_image handles indexed/alpha/grayscale files
end
sequence=prepare_image_sequence(images,[0 40 85 130],info);
load_image_sequence(model,sequence); output=sim(model);
updates=output.yout.getElement('resultUpdated').Values;
ids=output.yout.getElement('resultID').Values;
assert(nnz(updates.Data)==4 && ids.Data(end)==uint64(4),'EI:EventCount','Expected exactly four inference completions.');
fprintf('Exactly four native requests at 0, 40, 85 and 130 simulation seconds.\n');
end
