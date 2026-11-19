#include "mex.h"
#include "ei_wrapper.h"
#include <windows.h>
#include <string>
#include <stdexcept>
#include <cstring>
namespace {
HMODULE module=nullptr; bool owner=false; ei_model_info info{};
using Init=int32_t(*)(const ei_options*); using Info=int32_t(*)(ei_model_info*,uint32_t);
using Infer=int32_t(*)(const uint8_t*,uint64_t,uint32_t,uint32_t,ei_result*,uint32_t);
using Last=int32_t(*)(ei_result*,uint32_t); using Simple=int32_t(*)(); using Error=int32_t(*)(char*,uint32_t);
Init initialize; Info metadata; Infer infer; Last last; Simple reset,terminate; Error error;
template<class T>T symbol(const char *name){auto p=GetProcAddress(module,name);if(!p)throw std::runtime_error(std::string("Missing C ABI symbol: ")+name);return reinterpret_cast<T>(p);}
std::string string(const mxArray *a){if(!mxIsChar(a))throw std::runtime_error("Expected a char vector");char *p=mxArrayToString(a);if(!p)throw std::runtime_error("Cannot decode text");std::string s(p);mxFree(p);return s;}
void cleanup(){if(module){if(owner && terminate)terminate();owner=false;FreeLibrary(module);module=nullptr;}}
void load(const std::string &path){if(module)return;module=LoadLibraryExA(path.c_str(),nullptr,LOAD_WITH_ALTERED_SEARCH_PATH);if(!module)throw std::runtime_error("Cannot load DLL (Windows error "+std::to_string(GetLastError())+"): "+path);
 try{initialize=symbol<Init>("ei_initialize");metadata=symbol<Info>("ei_get_model_info");infer=symbol<Infer>("ei_infer_image");last=symbol<Last>("ei_get_result");reset=symbol<Simple>("ei_reset");terminate=symbol<Simple>("ei_terminate");error=symbol<Error>("ei_get_last_error");
 if(metadata(&info,sizeof(info))<0 || info.abi_version!=EI_ABI_VERSION || info.class_count!=1)throw std::runtime_error("Expected a single-class model with compatible ABI version");}
 catch(...){FreeLibrary(module);module=nullptr;throw;}}
void check(int32_t s){if(s<0){char e[1024]={};error(e,sizeof(e));throw std::runtime_error(std::to_string(s)+": "+e);}}
mxArray *number(double x){return mxCreateDoubleScalar(x);}
mxArray *make_info(){const char *names[]={"width","height","channels","classCount","lastLayer","sdkThreshold","labels"};auto a=mxCreateStructMatrix(1,1,7,names);
 mxSetField(a,0,"width",number(info.width));mxSetField(a,0,"height",number(info.height));mxSetField(a,0,"channels",number(info.channels));mxSetField(a,0,"classCount",number(info.class_count));mxSetField(a,0,"lastLayer",number(info.last_layer));mxSetField(a,0,"sdkThreshold",number(info.exported_threshold));
 auto labels=mxCreateCellMatrix(1,info.class_count);for(uint32_t i=0;i<info.class_count;i++)mxSetCell(labels,i,mxCreateString(info.labels[i]));mxSetField(a,0,"labels",labels);return a;}
mxArray *make_result(const ei_result &r){const char *names[]={"label","classIndex","confidence","detected","objectClass","status","boundingBox","elapsedMs","resultID","detectionCount","errorMessage"};
 auto a=mxCreateStructMatrix(1,1,11,names);mxSetField(a,0,"label",mxCreateString(r.detected&&r.class_index==0?info.labels[0]:""));
 mxSetField(a,0,"classIndex",number(r.class_index));mxSetField(a,0,"confidence",number(r.confidence));mxSetField(a,0,"detected",mxCreateLogicalScalar(r.detected!=0));mxSetField(a,0,"objectClass",mxCreateLogicalScalar(r.object_class!=0));mxSetField(a,0,"status",number(r.status));
 auto box=mxCreateDoubleMatrix(1,4,mxREAL);double *b=mxGetDoubles(box);b[0]=r.x;b[1]=r.y;b[2]=r.width;b[3]=r.height;mxSetField(a,0,"boundingBox",box);mxSetField(a,0,"elapsedMs",number(r.elapsed_ms));
 auto id=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);*static_cast<uint64_t*>(mxGetData(id))=r.result_id;mxSetField(a,0,"resultID",id);mxSetField(a,0,"detectionCount",number(r.detection_count));char e[1024]={};error(e,sizeof(e));mxSetField(a,0,"errorMessage",mxCreateString(r.status<0?e:""));return a;}
}
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]){
 try{
  if(nrhs<1 || nlhs>1)throw std::runtime_error("One command and at most one output required");
  const auto command=string(prhs[0]);
  if(command=="unload"){if(owner){check(terminate());owner=false;}cleanup();return;}
  if(!module){if(nrhs<2)throw std::runtime_error("DLL path required for initial call");load(string(prhs[1]));static bool registered=false;if(!registered){mexAtExit(cleanup);registered=true;}}
  mxArray *output=nullptr;
  if(command=="info")output=make_info();
  else if(command=="initialize"){
   if(nrhs!=7)throw std::runtime_error("initialize requires DLL path, threshold, false index, max detections, debug, timeout");
   ei_options p{float(mxGetScalar(prhs[2])),uint32_t(mxGetScalar(prhs[3])),uint32_t(mxGetScalar(prhs[4])),uint32_t(mxGetScalar(prhs[5])),mxGetScalar(prhs[6])};check(initialize(&p));owner=true;output=make_info();
  }else if(command=="infer"){
   if(nrhs!=2)throw std::runtime_error("infer requires one RGB uint8 array");auto image=prhs[1];
   const auto nd=mxGetNumberOfDimensions(image);
   const bool shape=info.channels==1 ? nd==2 : nd==3 && mxGetDimensions(image)[2]==3;
   if(!mxIsUint8(image)||mxIsComplex(image)||mxIsSparse(image)||!shape)throw std::runtime_error("Expected model-ready uint8 image: H x W grayscale or H x W x 3 RGB. Use run_ei_inference for automatic conversion.");
   const auto *dims=mxGetDimensions(image);ei_result r{};r.class_index=-1;
   const auto s=infer(static_cast<const uint8_t*>(mxGetData(image)),mxGetNumberOfElements(image),uint32_t(dims[0]),uint32_t(dims[1]),&r,sizeof(r));
   if(s<0){r.status=s;r.detected=0;r.confidence=0;r.object_class=0;r.class_index=-1;}output=make_result(r);
  }else if(command=="last"){ei_result r{};r.class_index=-1;auto s=last(&r,sizeof(r));if(s<0)r.status=s;output=make_result(r);}
  else if(command=="reset"){check(reset());}
  else if(command=="close"){if(owner){check(terminate());owner=false;}}
  else throw std::runtime_error("Unknown gateway command");
  if(output){if(nlhs)plhs[0]=output;else mxDestroyArray(output);}
 }catch(const std::exception &e){mexErrMsgIdAndTxt("EI:Native","%s",e.what());}
 catch(...){mexErrMsgIdAndTxt("EI:Native","Unknown gateway exception");}
}
