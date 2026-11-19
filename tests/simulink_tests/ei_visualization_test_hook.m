function output=ei_visualization_test_hook(action,model)
% Test-only observer of the optional viewer while the simulation is running.
persistent listener records
output=[];
switch action
    case 'attach'
        records=zeros(0,3);
        listener=add_exec_event_listener([model '/Viewer'],'PostOutputs',@observe);
    case 'detach'
        listener=[];
    case 'records'
        output=records;
end
    function observe(block,~)
        if block.OutputPort(6).Data
            assert(strcmp(get_param(model,'SimulationStatus'),'running'));
            records(end+1,:)=[block.CurrentTime double(block.OutputPort(5).Data) ...
                double(block.OutputPort(7).Data)];
        end
    end
end
