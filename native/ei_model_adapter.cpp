#include "ei_model_adapter.h"
#include "ei_image_adapter.h"
#include "ei_detection_selector.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include <cstring>
#include <cmath>
#include <limits>
// Defined by our guarded build-copy patch of the SDK. No network or second detector.
namespace ei_bridge {
int32_t model_info(ei_model_info &m, std::string &err) {
 m={}; m.abi_version=EI_ABI_VERSION;
 m.width=EI_CLASSIFIER_INPUT_WIDTH; m.height=EI_CLASSIFIER_INPUT_HEIGHT;
 m.class_count=EI_CLASSIFIER_LABEL_COUNT;
 m.last_layer=EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER;
 m.exported_threshold=EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD;
#if EI_CLASSIFIER_LABEL_COUNT != 1 || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA || EI_CLASSIFIER_OBJECT_DETECTION != 1 || EI_CLASSIFIER_COMPILED != 0 || EI_CLASSIFIER_INFERENCING_ENGINE != EI_CLASSIFIER_TFLITE
 err="Requires a single-class camera object detector with non-EON TensorFlow Lite Micro export";
 return EI_UNSUPPORTED_MODEL;
#else
 const auto *impulse=ei_default_impulse.impulse;
 if(!impulse || impulse->dsp_blocks_size!=1 || !impulse->dsp_blocks ||
    impulse->dsp_blocks[0].extract_fn!=&extract_image_features || !impulse->dsp_blocks[0].config) {
  err="Expected a single image DSP block"; return EI_UNSUPPORTED_MODEL;
 }
 const auto *dsp=static_cast<const ei_dsp_config_image_t*>(impulse->dsp_blocks[0].config);
 if(!dsp->channels) {err="Missing image DSP channels";return EI_UNSUPPORTED_MODEL;}
 if(std::strcmp(dsp->channels,"Grayscale")==0) m.channels=1;
 else if(std::strcmp(dsp->channels,"RGB")==0) m.channels=3;
 else {err="Only RGB or Grayscale image DSP input is supported";return EI_UNSUPPORTED_MODEL;}
 for(size_t c=0;c<EI_CLASSIFIER_LABEL_COUNT;c++) {
  const char *label=ei_classifier_inferencing_categories[c];
  if(!label || !*label || std::strlen(label)>=EI_LABEL_CAPACITY) {err="Empty or oversized class label";return EI_UNSUPPORTED_MODEL;}
  std::strcpy(m.labels[c],label);
 }
 return EI_READY;
#endif
}
int32_t model_init(std::string &err) {
 ei_model_info info{}; auto s=model_info(info,err); if(s<0) return s;
 run_classifier_init();
 // Preallocate the persistent TFLM interpreter without invoking the network.
 if(ei_default_impulse.impulse->learning_blocks_size!=1) {err="Only one learning block is supported";return EI_UNSUPPORTED_MODEL;}
 auto *cfg=static_cast<ei_learning_block_config_tflite_graph_t*>(ei_default_impulse.impulse->learning_blocks[0].config);
 TfLiteTensor *input=nullptr; std::vector<TfLiteTensor*> outputs(cfg->output_tensors_size);
 tflite::MicroInterpreter *interpreter=nullptr; uint64_t start=0; void *profiler=nullptr;
 ei_unique_ptr_t arena(nullptr,ei_aligned_free);
 auto rc=inference_tflite_setup(cfg,&start,&input,outputs.data(),&interpreter,arena,&profiler);
 if(rc!=EI_IMPULSE_OK) {err="TensorFlow Lite Micro initialization failed: "+std::to_string(rc);return EI_INFERENCE_FAILED;}
 return EI_READY;
}
void model_close() { run_classifier_deinit(); ei_default_impulse.state.reset(); ei_bridge_release_tflm(); }
int32_t model_infer(const uint8_t *rgb,const ei_options &opt,ei_result &r,std::string &err) {
 const auto *dsp=static_cast<const ei_dsp_config_image_t*>(ei_default_impulse.impulse->dsp_blocks[0].config);
 const uint32_t channels=std::strcmp(dsp->channels,"Grayscale")==0?1u:3u;
 signal_t signal; signal.total_length=size_t(EI_CLASSIFIER_INPUT_WIDTH)*EI_CLASSIFIER_INPUT_HEIGHT;
 signal.get_data=[rgb,channels](size_t offset,size_t length,float *out)->int {
  const size_t pixels=size_t(EI_CLASSIFIER_INPUT_WIDTH)*EI_CLASSIFIER_INPUT_HEIGHT;
  if(offset>pixels || length>pixels-offset || !out) return -1;
  for(size_t i=0;i<length;i++) out[i]=float(packed_pixel(rgb,EI_CLASSIFIER_INPUT_HEIGHT,EI_CLASSIFIER_INPUT_WIDTH,offset+i,channels));
  return 0;
 };
 ei_impulse_result_t result{};
 auto code=run_classifier(&signal,&result,opt.debug_log!=0);
 if(code!=EI_IMPULSE_OK) {err="run_classifier returned "+std::to_string(code);return EI_INFERENCE_FAILED;}
 if(result.bounding_boxes_count>opt.max_detections) {err="SDK detection count exceeds configured bound (including padding)";return EI_TOO_MANY_DETECTIONS;}
 if(result.bounding_boxes_count && !result.bounding_boxes) {err="SDK returned a null box array";return EI_INFERENCE_FAILED;}
 Selector select(opt); uint32_t valid=0;
 for(size_t i=0;i<result.bounding_boxes_count;i++) {
  const auto &b=result.bounding_boxes[i]; int32_t c=-1;
  for(int32_t j=0;j<EI_CLASSIFIER_LABEL_COUNT;j++) if(b.label && std::strcmp(b.label,ei_classifier_inferencing_categories[j])==0) c=j;
  if(!std::isfinite(b.value) || b.value<=0 || b.value>1) continue;
  if(c<0 || !b.width || !b.height || b.x>=EI_CLASSIFIER_INPUT_WIDTH || b.y>=EI_CLASSIFIER_INPUT_HEIGHT ||
     uint64_t(b.x)+b.width>EI_CLASSIFIER_INPUT_WIDTH || uint64_t(b.y)+b.height>EI_CLASSIFIER_INPUT_HEIGHT) {
   err="SDK returned an invalid class or bounding box";return EI_INFERENCE_FAILED;
  }
  ++valid; select.consider({c,b.value,b.x,b.y,b.width,b.height,i});
 }
 r.detection_count=valid; select.finish(r); return r.status;
}
}
