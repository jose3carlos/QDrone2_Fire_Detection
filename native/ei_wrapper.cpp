#include "ei_wrapper.h"
#include "ei_model_adapter.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <exception>
namespace {
std::mutex gate; bool initialized=false, simulation=false; ei_options options{};
uint64_t sequence=0; ei_result last{}; thread_local std::string error;
int32_t fail(int32_t s,const char *e) noexcept {try {error=e;}catch(...){}return s;}
void clear_result(ei_result &r,int32_t s) {r={};r.class_index=-1;r.status=s;}
int32_t initialize_locked(const ei_options *p) {
 if(!p || !std::isfinite(p->min_confidence) || p->min_confidence<0 || p->min_confidence>1 ||
    p->false_class_index!=0 || !p->max_detections || p->max_detections>1000000 ||
    !std::isfinite(p->timeout_ms) || p->timeout_ms<0) return fail(EI_INVALID_CONFIG,"Invalid initialization options");
 if(initialized) return fail(EI_BUSY,"Terminate existing owner before initialization");
 int32_t s;
 try {s=ei_bridge::model_init(error);}
 catch(...) {try {ei_bridge::model_close();}catch(...){}throw;}
 if(s<0) {ei_bridge::model_close();return s;}
 options=*p; initialized=true; sequence=0; clear_result(last,EI_READY);error.clear();return EI_READY;
}
int32_t infer_locked(const uint8_t *rgb,uint64_t bytes,uint32_t h,uint32_t w,ei_result *r,uint32_t cap) {
 if(!r || cap<sizeof(ei_result)) return fail(EI_INVALID_CONFIG,"Result buffer is too small");
 clear_result(*r,EI_READY);
 if(!initialized) {r->status=EI_NOT_INITIALIZED;return fail(r->status,"Model is not initialized");}
 r->result_id=++sequence;
 ei_model_info m{}; ei_bridge::model_info(m,error);
 if(!rgb || h!=m.height || w!=m.width || bytes!=uint64_t(h)*w*m.channels) {
  r->status=EI_INVALID_IMAGE;last=*r;return fail(r->status,"Expected exact model H x W x channels uint8 buffer (1 grayscale or 3 RGB)");
 }
 error.clear(); const auto start=std::chrono::steady_clock::now();
 int32_t status=EI_NATIVE_FAILURE;
 try {status=ei_bridge::model_infer(rgb,options,*r,error);}
 catch(const std::exception &e) {error=e.what();}
 catch(...) {error="Unknown C++ inference exception";}
 r->elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 if(options.timeout_ms>0 && r->elapsed_ms>options.timeout_ms) {status=EI_TIMEOUT;error="Inference exceeded configured deadline; synchronous call has now completed";}
 if(status<0) {const auto id=r->result_id;const auto ms=r->elapsed_ms;clear_result(*r,status);r->result_id=id;r->elapsed_ms=ms;}
 r->status=status; last=*r;return status;
}
}
#define EI_GUARD_BEGIN try { std::unique_lock<std::mutex> lock(gate,std::try_to_lock); if(!lock.owns_lock()) return fail(EI_BUSY,"Native engine is busy");
#define EI_GUARD_END } catch(const std::exception &e) { return fail(EI_NATIVE_FAILURE,e.what()); } catch(...) { return fail(EI_NATIVE_FAILURE,"Unknown native exception"); }
extern "C" {
int32_t ei_initialize(const ei_options *p) {EI_GUARD_BEGIN if(simulation)return fail(EI_BUSY,"Simulation owns engine");return initialize_locked(p); EI_GUARD_END}
int32_t ei_get_model_info(ei_model_info *m,uint32_t cap) {EI_GUARD_BEGIN if(!m || cap<sizeof(*m))return fail(EI_INVALID_CONFIG,"Metadata buffer too small");return ei_bridge::model_info(*m,error);EI_GUARD_END}
int32_t ei_infer_image(const uint8_t *p,uint64_t n,uint32_t h,uint32_t w,ei_result *r,uint32_t cap) {
 if(r && cap>=sizeof(*r))clear_result(*r,EI_BUSY);
 EI_GUARD_BEGIN if(simulation){if(r && cap>=sizeof(*r))clear_result(*r,EI_BUSY);return fail(EI_BUSY,"Simulation owns engine");}return infer_locked(p,n,h,w,r,cap);EI_GUARD_END
}
int32_t ei_get_result(ei_result *r,uint32_t cap) {EI_GUARD_BEGIN if(!r || cap<sizeof(*r))return EI_INVALID_CONFIG;if(!initialized){clear_result(*r,EI_NOT_INITIALIZED);return EI_NOT_INITIALIZED;}*r=last;return r->status;EI_GUARD_END}
int32_t ei_reset(void) {EI_GUARD_BEGIN if(simulation)return EI_BUSY;if(!initialized)return EI_NOT_INITIALIZED;clear_result(last,EI_READY);sequence=0;error.clear();return EI_READY;EI_GUARD_END}
int32_t ei_terminate(void) {EI_GUARD_BEGIN if(simulation)return EI_BUSY;if(initialized)ei_bridge::model_close();initialized=false;clear_result(last,EI_READY);return EI_READY;EI_GUARD_END}
int32_t ei_get_last_error(char *p,uint32_t n) {try {if(!p || !n)return EI_INVALID_CONFIG;auto k=error.size()<n-1?error.size():n-1;std::memcpy(p,error.data(),k);p[k]=0;return EI_READY;}catch(...){return EI_NATIVE_FAILURE;}}
int32_t ei_sim_start(float t,uint32_t f,uint32_t n,uint32_t d,double timeout) {EI_GUARD_BEGIN if(simulation || initialized)return fail(EI_BUSY,"Engine already owned: close standalone model before simulation");ei_options p{t,f,n,d,timeout};auto s=initialize_locked(&p);if(s>=0)simulation=true;return s;EI_GUARD_END}
void ei_sim_stop(void) {try {std::lock_guard<std::mutex> lock(gate);if(simulation){ei_bridge::model_close();initialized=false;simulation=false;clear_result(last,EI_READY);}}catch(...){}}
int32_t ei_sim_infer(const uint8_t *p,uint64_t n,uint32_t h,uint32_t w,float *c,uint8_t *o,uint8_t *d,uint64_t *id) {
 if(!c || !o || !d || !id)return EI_INVALID_CONFIG;
 *c=0;*o=0;*d=0;*id=0;
 EI_GUARD_BEGIN if(!simulation)return fail(EI_NOT_INITIALIZED,"Simulation initialization did not succeed");ei_result r{};
 auto s=infer_locked(p,n,h,w,&r,sizeof(r));*c=r.confidence;*o=uint8_t(r.object_class);*d=uint8_t(r.detected);*id=r.result_id;return s;EI_GUARD_END
}
}
