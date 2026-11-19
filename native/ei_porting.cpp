#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdarg>
EI_IMPULSE_ERROR ei_run_impulse_check_canceled(){return EI_IMPULSE_OK;}
EI_IMPULSE_ERROR ei_sleep(int32_t ms){std::this_thread::sleep_for(std::chrono::milliseconds(ms>0?ms:0));return EI_IMPULSE_OK;}
uint64_t ei_read_timer_us(){return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());}
uint64_t ei_read_timer_ms(){return ei_read_timer_us()/1000;}
void ei_printf(const char *format,...){va_list args;va_start(args,format);std::vfprintf(stderr,format,args);va_end(args);}
void ei_printf_float(float x){std::fprintf(stderr,"%f",double(x));}
void ei_putchar(char c){std::fputc(c,stderr);}
char ei_getchar(){return 0;}
void ei_serial_set_baudrate(int){}
void *ei_malloc(size_t n){return std::malloc(n);}
void *ei_calloc(size_t n,size_t s){return std::calloc(n,s);}
void ei_free(void *p){std::free(p);}
extern "C" void DebugLog(const char *s){ei_printf("%s",s);}
