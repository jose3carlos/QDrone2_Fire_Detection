function ei_test_live_hook(action,model,image)
% Supported simulation execution listener injects a new frame DURING simulation.
persistent listener submitted
switch action
    case 'attach'
        submitted=false;
        listener=add_exec_event_listener([model '/Live Image Mailbox'],'PostOutputs',@inject); %#ok<NASGU>
    case 'detach'
        listener=[];
end
    function inject(block,~)
        if ~submitted && block.CurrentTime>=0.4
            r=submit_live_image(image);assert(r.accepted);submitted=true;
        end
    end
end
