function varargout = ei_live_mailbox(action,varargin)
% MATLAB simulation thread only. One complete pending frame, reject when full.
% A Level-2 MATLAB S-function polls this mailbox; From Workspace is not used live.
persistent pending ticket counter
if isempty(counter), counter=uint64(0); end
switch action
    case 'reset'
        pending=[]; ticket=uint64(0); counter=uint64(0);
    case 'put'
        accepted=isempty(pending);
        if accepted
            pending=varargin{1}; counter=counter+uint64(1); ticket=counter;
        end
        varargout={accepted,ticket};
    case 'take'
        varargout={pending,ticket}; pending=[];
    case 'pending'
        varargout={~isempty(pending)};
    otherwise, error('EI:Mailbox','Unknown mailbox action.');
end
end
