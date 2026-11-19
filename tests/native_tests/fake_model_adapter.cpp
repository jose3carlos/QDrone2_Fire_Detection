// Test-only deterministic backend for Simulink stale-result/error transitions.
// This file is linked exclusively into ei_contract_fixture.dll, never inference.
#include "ei_model_adapter.h"
#include "model-parameters/model_metadata.h"
#include <cstring>
namespace ei_bridge {
int32_t model_info(ei_model_info &i,std::string &){i={};i.abi_version=1;i.width=EI_CLASSIFIER_INPUT_WIDTH;i.height=EI_CLASSIFIER_INPUT_HEIGHT;i.channels=EI_CLASSIFIER_NN_INPUT_FRAME_SIZE/(EI_CLASSIFIER_INPUT_WIDTH*EI_CLASSIFIER_INPUT_HEIGHT);i.class_count=1;i.last_layer=2;i.exported_threshold=.5f;std::strcpy(i.labels[0],"Class_A");return EI_READY;}
int32_t model_init(std::string &){return EI_READY;}
void model_close(){}
int32_t model_infer(const uint8_t *image,const ei_options &o,ei_result &r,std::string &error){
 if(image[0]==255){error="Injected inference failure for lifecycle test";return EI_INFERENCE_FAILED;}
 if(image[0]==0){r.class_index=-1;r.status=EI_NO_DETECTION;return EI_NO_DETECTION;}
 r.class_index=int32_t(o.false_class_index);r.confidence=91;r.detected=1;r.object_class=0;
 r.width=8;r.height=8;r.detection_count=1;r.status=EI_SUCCESS;return EI_SUCCESS;
}
}
