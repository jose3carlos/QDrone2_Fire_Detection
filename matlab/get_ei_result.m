function result = get_ei_result()
state=ei_session('get');assert(~isempty(state),'EI:NotLoaded','Call load_ei_model first.');
result=feval(state.artifact.gateway,'last');
end
