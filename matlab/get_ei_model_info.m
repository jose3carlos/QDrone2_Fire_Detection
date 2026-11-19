function info = get_ei_model_info()
state=ei_session('get');
assert(~isempty(state),'EI:NotLoaded','Call load_ei_model first.');
info=state.info;
end
