#include "ei_wrapper.h"
#include <cstdio>
#include <stdexcept>
#include <vector>
void require(bool b){if(!b){char error[1024]={};ei_get_last_error(error,sizeof(error));throw std::runtime_error(error[0]?error:"Native integration assertion failed");}}
int main(){
 ei_model_info m{};require(ei_get_model_info(&m,sizeof(m))==EI_READY);
 require(m.class_count==1 && m.width>0 && m.height>0 && m.labels[0][0]);
 require(m.channels==1 || m.channels==3);
 ei_options o{.5f,0,4096,0,0};ei_result r{};std::vector<uint8_t> pixels(size_t(m.width)*m.height*m.channels,0);
 require(ei_infer_image(pixels.data(),pixels.size(),m.height,m.width,&r,sizeof(r))==EI_NOT_INITIALIZED);
 require(ei_initialize(&o)==EI_READY);
 require(ei_infer_image(pixels.data(),pixels.size()-1,m.height,m.width,&r,sizeof(r))==EI_INVALID_IMAGE && !r.detected);
 require(ei_reset()==EI_READY);
 for(int i=0;i<3;i++){
  auto s=ei_infer_image(pixels.data(),pixels.size(),m.height,m.width,&r,sizeof(r));
  require(s==EI_SUCCESS || s==EI_NO_DETECTION);require(r.result_id==uint64_t(i+1));
  require(r.confidence>=0 && r.confidence<=100 && r.elapsed_ms>0);
  if(r.detected) require(r.class_index==0 && !r.object_class && r.width>0 && r.height>0 && r.x+r.width<=m.width && r.y+r.height<=m.height);
  std::printf("Actual model %ux%u [%s]: status=%d confidence=%.2f time=%.3f ms id=%llu\n",m.width,m.height,m.labels[0],s,r.confidence,r.elapsed_ms,(unsigned long long)r.result_id);
 }
 require(ei_terminate()==EI_READY);require(ei_initialize(&o)==EI_READY);require(ei_terminate()==EI_READY);
 std::puts("Native lifecycle/inference integration passed (synthetic black image; no accuracy claim)");
}
