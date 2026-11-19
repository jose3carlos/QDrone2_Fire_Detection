function ei_patch_sdk(folder)
% Modify ONLY a disposable extracted build copy, preserving the original ZIP.
% A new SDK with a different setup signature fails explicitly for review.
file = fullfile(folder,'edge-impulse-sdk','classifier','inferencing_engines','tflite_micro.h');
text = fileread(file);
if contains(text,'ei_tflm_cache.inc'), return; end
start = strfind(text,'static EI_IMPULSE_ERROR inference_tflite_setup(');
marker = regexp(text,'/\*\*\s*\n \* Run TFLite model','start');
assert(numel(start)==1 && numel(marker)==1 && marker>start, ...
    'EI:UnsupportedSDK','TFLM setup signature changed; cannot safely apply persistent-interpreter patch.');
assert(contains(text,'ei_unique_ptr_t& p_tensor_arena') && ...
    contains(text,'AllocateTensors(true)'),'EI:UnsupportedSDK','Unsupported TFLM lifetime implementation.');
text = [text(1:start-1) '#include "ei_tflm_cache.inc"' newline newline text(marker:end)];
text = strrep(text,'delete interpreter;','/* Persistent interpreter is released by ei_bridge_release_tflm(). */');
f=fopen(file,'w'); assert(f>=0); cleaner=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,text,'char');
end
