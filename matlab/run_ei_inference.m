function result = run_ei_inference(image)
% One explicit call is one independent synchronous inference request.
ei_assert_stopped(); state=ei_session('get');
if isempty(state), load_ei_model(); state=ei_session('get'); end
cfg=config();
assert(strcmp(char(cfg.modelPath),state.artifact.modelPath),'EI:ModelChanged','Configuration changed. Close and reload the selected model.');
image=prepare_image(image,state.info);
result=feval(state.artifact.gateway,'infer',image);
end
