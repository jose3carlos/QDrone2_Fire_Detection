function value = ei_session(action,value)
% Session bookkeeping contains no inference implementation.
persistent state
if nargin<2, value=[]; end
switch action
    case 'get', value=state;
    case 'set', state=value;
    case 'clear', state=[];
    otherwise, error('EI:Session','Unknown session action.');
end
end
