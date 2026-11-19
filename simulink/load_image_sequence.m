function load_image_sequence(model,sequence)
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'EI:Running','Stop simulation before loading a pre-recorded sequence.');
workspace=get_param(model,'ModelWorkspace');
assignin(workspace,'eiImageSequence',sequence.image);
assignin(workspace,'eiNewImageSequence',sequence.newImage);
set_param(model,'StopTime',num2str(sequence.stopTime,17));
end
