function result = test_single_image(file)
% No argument: structural smoke test with a synthetic black image.
setup_project(false); load_ei_model(); info=get_ei_model_info();
if nargin<1, image=zeros(info.height,info.width,info.channels,'uint8'); else, image=file; end
result=run_ei_inference(image); disp(result);
assert(result.status>=0,'EI:Inference','Inference failed: %s',result.errorMessage);
end
