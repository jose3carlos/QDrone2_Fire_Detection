// Included only inside the build copy of SDK tflite_micro.h.
// Replaces interpreter setup/lifetime, preserving the SDK DSP, input quantization,
// Invoke, tensor extraction and model-specific postprocessing.
static ei_unique_ptr_t ei_bridge_arena(nullptr, ei_aligned_free);
static std::unique_ptr<tflite::MicroInterpreter> ei_bridge_interpreter;
static const ei_config_tflite_graph_t *ei_bridge_graph=nullptr;
static void ei_bridge_release_tflm() {
 ei_bridge_interpreter.reset(); ei_bridge_arena.reset(); ei_bridge_graph=nullptr;
}
static EI_IMPULSE_ERROR inference_tflite_setup(
 ei_learning_block_config_tflite_graph_t *cfg,uint64_t *start,
 TfLiteTensor **input,TfLiteTensor **outputs,tflite::MicroInterpreter **interpreter,
 ei_unique_ptr_t &call_arena,void **profiler) {
 *start=ei_read_timer_us(); *profiler=nullptr;
 auto *graph=static_cast<ei_config_tflite_graph_t*>(cfg->graph_config);
 if(!ei_bridge_interpreter || graph!=ei_bridge_graph) {
  ei_bridge_release_tflm();
  const auto *model=tflite::GetModel(graph->model);
  if(model->version()!=TFLITE_SCHEMA_VERSION) return EI_IMPULSE_TFLITE_ERROR;
  ei_bridge_arena=ei_unique_ptr_t(ei_aligned_calloc(16,graph->arena_size),ei_aligned_free);
  if(!ei_bridge_arena) return EI_IMPULSE_TFLITE_ARENA_ALLOC_FAILED;
#ifdef EI_TFLITE_RESOLVER
  EI_TFLITE_RESOLVER
#else
  static tflite::AllOpsResolver resolver;
#endif
  ei_bridge_interpreter.reset(new tflite::MicroInterpreter(model,resolver,
    static_cast<uint8_t*>(ei_bridge_arena.get()),graph->arena_size,nullptr,nullptr));
  if(ei_bridge_interpreter->AllocateTensors(true)!=kTfLiteOk) {
   ei_bridge_release_tflm();return EI_IMPULSE_TFLITE_ERROR;
  }
  ei_bridge_graph=graph;
 }
 // Images are independent; variable tensors must not carry history across events.
 if(ei_bridge_interpreter->Reset()!=kTfLiteOk) return EI_IMPULSE_TFLITE_ERROR;
 *input=ei_bridge_interpreter->input(0);
 for(uint8_t i=0;i<cfg->output_tensors_size;i++) outputs[i]=ei_bridge_interpreter->output(cfg->output_tensors_indices[i]);
 *interpreter=ei_bridge_interpreter.get();
 call_arena=ei_unique_ptr_t(nullptr,ei_aligned_free); // cache owns the arena
 return EI_IMPULSE_OK;
}
