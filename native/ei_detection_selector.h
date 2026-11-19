#pragma once
#include "ei_wrapper.h"
#include <cstddef>
namespace ei_bridge {
struct Detection { int32_t model_class; float confidence; uint32_t x,y,w,h; size_t index; };
// Accumulator inspects ALL SDK detections; bounds never silently truncate selection.
class Selector {
 ei_options options_; bool found_=false; Detection best_{};
public:
 explicit Selector(ei_options options):options_(options){}
 void consider(const Detection &d);
 void finish(ei_result &r) const;
};
}
