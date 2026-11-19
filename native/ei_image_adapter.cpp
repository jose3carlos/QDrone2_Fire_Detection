#include "ei_image_adapter.h"
namespace ei_bridge {
uint32_t packed_pixel(const uint8_t *p, uint32_t h, uint32_t w, size_t n, uint32_t channels) {
 const size_t plane=size_t(h)*w, k=(n/w)+(n%w)*h;
 if(channels==1) return uint32_t(p[k])*0x010101u; // SDK still expects packed RGB pixels
 return (uint32_t(p[k])<<16)|(uint32_t(p[k+plane])<<8)|p[k+2*plane];
}
}
