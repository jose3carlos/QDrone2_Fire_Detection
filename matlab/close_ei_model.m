function close_ei_model()
state=ei_session('get');
if ~isempty(state)
    feval(state.artifact.gateway,'close');
    feval(state.artifact.gateway,'unload');
    ei_session('clear');
end
end
