#include "ei_detection_selector.h"
#include "ei_image_adapter.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <cstdio>
#include <vector>
void require(bool b){if(!b)throw std::runtime_error("Algorithm test assertion failed");}
ei_result select(std::vector<ei_bridge::Detection> d,float threshold=.5f){ei_options o{threshold,0,100,0,0};ei_bridge::Selector s(o);ei_result r{};for(auto x:d)s.consider(x);s.finish(r);return r;}
ei_bridge::Detection box(int c,float p,size_t i=0){return {c,p,0,0,8,8,i};}
int main(){
 auto r=select({box(0,.92f)});require(r.detected && !r.object_class && r.class_index==0 && std::fabs(r.confidence-92)<.001);
 r=select({box(1,.99f)});require(!r.detected); // reject any second class
 r=select({box(1,.99f),box(0,.78f)});require(r.class_index==0 && r.confidence==78);
 r=select({});require(!r.detected && r.confidence==0 && r.class_index==-1);
 r=select({box(0,.6f),box(0,.95f,1),box(0,.8f,2)});require(r.selected_detection==1);
 r=select({box(0,.49f),box(0,.1f)});require(!r.detected);
 r=select({box(0,.9f,2),box(0,.9f,1)});require(r.class_index==0 && r.selected_detection==1);
 r=select({box(0,std::numeric_limits<float>::quiet_NaN()),box(0,2),box(-1,.9f),box(0,-.1f),box(0,0)},0);require(!r.detected);
 r=select({box(0,.5f)});require(r.detected);
 // Asymmetric 2x3 image catches row/column and channel ordering mistakes.
 uint8_t rgb[18]={1,2,3,4,5,6,11,12,13,14,15,16,21,22,23,24,25,26};
 require(ei_bridge::packed_pixel(rgb,2,3,0)==0x010b15);
 require(ei_bridge::packed_pixel(rgb,2,3,1)==0x030d17);
 require(ei_bridge::packed_pixel(rgb,2,3,3)==0x020c16);
 require(ei_bridge::packed_pixel(rgb,2,3,5)==0x06101a);
 const uint8_t gray[6]={1,2,3,4,5,255};
 require(ei_bridge::packed_pixel(gray,2,3,1,1)==0x030303);
 require(ei_bridge::packed_pixel(gray,2,3,3,1)==0x020202);
 require(ei_bridge::packed_pixel(gray,2,3,5,1)==0xffffff);
 std::puts("Native algorithm tests passed: confidence cases, ties, invalid scores, RGB/grayscale column-major layout");
}
