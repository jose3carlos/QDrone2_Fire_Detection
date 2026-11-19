function reset_ei_inference()
ei_assert_stopped(); state=ei_session('get');
assert(~isempty(state),'EI:NotLoaded','Call load_ei_model first.');
feval(state.artifact.gateway,'reset');
end
