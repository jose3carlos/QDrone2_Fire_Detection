function info = inspect_model(folder)
% Inspect actual generated headers. Runtime metadata is cross-checked after build.
required = {'model-parameters/model_metadata.h','model-parameters/model_variables.h', ...
    'edge-impulse-sdk/classifier/ei_run_classifier.h', ...
    'edge-impulse-sdk/classifier/inferencing_engines/tflite_micro.h', ...
    'edge-impulse-sdk/tensorflow/lite/micro/micro_interpreter.h','tflite-model/tflite-resolver.h'};
for i=1:numel(required)
    assert(isfile(fullfile(folder,required{i})),'EI:IncompleteExport','Missing export file: %s',required{i});
end
header = fileread(fullfile(folder,required{1}));
info.width = number(header,'EI_CLASSIFIER_INPUT_WIDTH');
info.height = number(header,'EI_CLASSIFIER_INPUT_HEIGHT');
info.classCount = number(header,'EI_CLASSIFIER_LABEL_COUNT');
info.compiled = number(header,'EI_CLASSIFIER_COMPILED');
info.sdkThreshold = number(header,'EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD');
info.arenaBytes = number(header,'EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE');
assert(info.width>0 && info.height>0 && info.classCount==1 && info.compiled==0 && info.arenaBytes>0, ...
    'EI:UnsupportedExport','Expected a single-class, non-EON image detector with a TFLM arena.');
assert(number(header,'EI_CLASSIFIER_OBJECT_DETECTION')==1 && ...
    ~isempty(regexp(header,'#define\s+EI_CLASSIFIER_INFERENCING_ENGINE\s+EI_CLASSIFIER_TFLITE\s','once')), ...
    'EI:UnsupportedExport','Export must use object detection with TensorFlow Lite Micro.');
assert(number(header,'EI_CLASSIFIER_OBJECT_TRACKING_ENABLED')==0, ...
    'EI:UnsupportedExport','Disable object tracking in the exported impulse.');
vars = fileread(fullfile(folder,required{2}));
channels=regexp(vars,'ei_dsp_config_image_t\s+\w+\s*=\s*\{[^}]*"(RGB|Grayscale)"','tokens');
assert(numel(channels)==1,'EI:UnsupportedChannels','Expected one RGB or Grayscale image DSP configuration.');
info.channelMode=channels{1}{1}; info.channels=3;
if strcmp(info.channelMode,'Grayscale'),info.channels=1;end
labels = regexp(vars,'ei_classifier_inferencing_categories[^\[]*\[\]\s*=\s*\{([^}]+)\}','tokens','once');
assert(~isempty(labels),'EI:UnsupportedExport','Cannot locate exported category array.');
info.labels = regexp(labels{1},'"([^"\\]+)"','tokens');
info.labels = cellfun(@(x)x{1},info.labels,'UniformOutput',false);
assert(numel(info.labels)==1,'EI:Labels','Exactly one foreground class label required.');
assert(all(cellfun(@(s)numel(unicode2native(s,'UTF-8'))<128,info.labels)),'EI:Labels','Class labels must fit 127 UTF-8 bytes.');
layer = regexp(header,'#define\s+EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER\s+(\w+)','tokens','once');
info.lastLayer = layer{1};
info.resizeMode = regexp(header,'#define\s+EI_CLASSIFIER_RESIZE_MODE\s+(\w+)','tokens','once');
info.resizeMode = info.resizeMode{1};
assert(ismember(info.resizeMode,{'EI_CLASSIFIER_RESIZE_FIT_SHORTEST','EI_CLASSIFIER_RESIZE_FIT_LONGEST', ...
    'EI_CLASSIFIER_RESIZE_SQUASH','EI_CLASSIFIER_RESIZE_NONE'}), ...
    'EI:UnsupportedResize','Unsupported export resize mode %s.',info.resizeMode);
modelFiles = dir(fullfile(folder,'tflite-model','*.cpp'));
assert(~isempty(modelFiles),'EI:IncompleteExport','No compiled TFLite model array found.');
info.root = folder;
end
function n = number(text,name)
t = regexp(text,['#define\s+' name '\s+([0-9.]+)'],'tokens','once');
assert(~isempty(t),'EI:UnsupportedSDK','Missing numeric metadata symbol: %s',name);
n = str2double(t{1});
end
