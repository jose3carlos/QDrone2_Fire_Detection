function value=ei_custom_validation_hook(action,model,artifact)
% Test-only execution observer: inspect results while simulation is running.
persistent listener observations activeArtifact
value=[];
switch action
    case 'attach'
        activeArtifact=artifact;
        addpath(artifact.gatewayPath);
        observations=zeros(0,4);
        listener=add_exec_event_listener([model '/FOMO Inference'],'PostOutputs',@observe);
    case 'results'
        value=observations;
    case 'detach'
        listener=[];
end
    function observe(block,~)
        assert(strcmp(get_param(model,'SimulationStatus'),'running'));
        result=feval(activeArtifact.gateway,'last',activeArtifact.dll);
        assert(result.status>=0,'Native engine failed during live simulation.');
        observations(end+1,:)=[block.CurrentTime double(result.resultID) ...
            double(block.OutputPort(1).Data) double(block.OutputPort(2).Data)];
    end
end
