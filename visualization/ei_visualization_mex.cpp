// Optional visualization gateway. Uses the existing simulation-owned engine.
// It never initializes, terminates, or changes ownership of that engine.
#include "mex.h"
#include "ei_wrapper.h"
#include <windows.h>
#include <string>
#include <stdexcept>
#include <cmath>
namespace {
HMODULE module=nullptr;
std::string loadedPath;
using Info=int32_t(*)(ei_model_info*,uint32_t);
using SimInfer=int32_t(*)(const uint8_t*,uint64_t,uint32_t,uint32_t,float*,uint8_t*,uint8_t*,uint64_t*);
using Last=int32_t(*)(ei_result*,uint32_t);
using Error=int32_t(*)(char*,uint32_t);
Info metadata=nullptr; SimInfer infer=nullptr; Last last=nullptr; Error error=nullptr;
ei_model_info info{};
std::string string(const mxArray *a) {
 if(!mxIsChar(a))throw std::runtime_error("Expected character-vector command/path");
 char *p=mxArrayToString(a);if(!p)throw std::runtime_error("Cannot decode text");
 std::string s(p);mxFree(p);return s;
}
template<class T>T symbol(const char *name) {
 auto p=GetProcAddress(module,name);
 if(!p)throw std::runtime_error(std::string("Missing existing native symbol: ")+name);
 return reinterpret_cast<T>(p);
}
void unload(){if(module)FreeLibrary(module);module=nullptr;loadedPath.clear();}
void load(const std::string &path) {
 if(module && path==loadedPath)return;
 unload(); module=LoadLibraryExA(path.c_str(),nullptr,LOAD_WITH_ALTERED_SEARCH_PATH);
 if(!module)throw std::runtime_error("Cannot load inference DLL, Windows error "+std::to_string(GetLastError()));
 try {
  metadata=symbol<Info>("ei_get_model_info");infer=symbol<SimInfer>("ei_sim_infer");
  last=symbol<Last>("ei_get_result");error=symbol<Error>("ei_get_last_error");
  if(metadata(&info,sizeof(info))<0 || info.abi_version!=EI_ABI_VERSION || info.class_count!=1)
   throw std::runtime_error("Incompatible single-class native ABI");
  loadedPath=path;
 }catch(...){unload();throw;}
}
void check(int32_t status) {
 if(status<0){char message[1024]={};error(message,sizeof(message));
  throw std::runtime_error(std::to_string(status)+": "+message);}
}
mxArray *number(double v){return mxCreateDoubleScalar(v);}
mxArray *make_result(const ei_result &r) {
 const char *names[]={"status","detected","confidence","boundingBox","resultID","label","detectionCount"};
 auto result=mxCreateStructMatrix(1,1,7,names);
 mxSetField(result,0,"status",number(r.status));
 mxSetField(result,0,"detected",mxCreateLogicalScalar(r.detected!=0));
 mxSetField(result,0,"confidence",number(r.confidence));
 auto box=mxCreateNumericMatrix(1,4,mxUINT32_CLASS,mxREAL);
 auto values=static_cast<uint32_t*>(mxGetData(box));
 values[0]=r.x;values[1]=r.y;values[2]=r.width;values[3]=r.height;
 mxSetField(result,0,"boundingBox",box);
 auto id=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);
 *static_cast<uint64_t*>(mxGetData(id))=r.result_id;mxSetField(result,0,"resultID",id);
 mxSetField(result,0,"label",mxCreateString(r.detected?info.labels[0]:""));
 mxSetField(result,0,"detectionCount",number(r.detection_count));return result;
}
}
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {
 try {
  if(nrhs<1 || nlhs>1)throw std::runtime_error("One command and at most one output required");
  const auto command=string(prhs[0]);
  if(command=="unload"){unload();return;}
  if(nrhs<2)throw std::runtime_error("DLL path required");
  load(string(prhs[1]));static bool registered=false;
  if(!registered){mexAtExit(unload);registered=true;}
  mxArray *out=nullptr;
  if(command=="info") {
   const char *names[]={"height","width","channels"};out=mxCreateStructMatrix(1,1,3,names);
   mxSetField(out,0,"height",number(info.height));mxSetField(out,0,"width",number(info.width));
   mxSetField(out,0,"channels",number(info.channels));
  } else if(command=="infer") {
   if(nrhs!=3)throw std::runtime_error("infer requires DLL path and one model-ready image");
   const auto image=prhs[2];
   if(info.channels!=1 || !mxIsUint8(image) || mxIsComplex(image) || mxIsSparse(image) ||
      mxGetNumberOfDimensions(image)!=2 || mxGetM(image)!=info.height || mxGetN(image)!=info.width)
    throw std::runtime_error("Expected exact model-ready grayscale uint8 H-by-W image");
   float confidence=0;uint8_t object=0,detected=0;uint64_t id=0;
   auto status=infer(static_cast<const uint8_t*>(mxGetData(image)),mxGetNumberOfElements(image),
       info.height,info.width,&confidence,&object,&detected,&id);
   check(status);
   ei_result result{};check(last(&result,sizeof(result)));
   // No other MATLAB block can run between these synchronous native calls.
   // Detect an unexpected external native caller rather than pairing a stale box.
   if(!id || result.result_id!=id || result.status!=status || result.detected!=detected ||
      result.confidence!=confidence || (status!=EI_SUCCESS && status!=EI_NO_DETECTION) ||
      !std::isfinite(confidence) || confidence<0 || confidence>100)
    throw std::runtime_error("Result changed before its bounding box could be copied");
   if(detected && (!result.width || !result.height || result.x>=info.width || result.y>=info.height ||
      uint64_t(result.x)+result.width>info.width || uint64_t(result.y)+result.height>info.height))
    throw std::runtime_error("Native bounding box is outside the inferred image");
   out=make_result(result);
  } else throw std::runtime_error("Unknown visualization gateway command");
  if(out){if(nlhs)plhs[0]=out;else mxDestroyArray(out);}
 }catch(const std::exception &e){mexErrMsgIdAndTxt("EI:VisualizationNative","%s",e.what());}
 catch(...){mexErrMsgIdAndTxt("EI:VisualizationNative","Unknown visualization error");}
}
