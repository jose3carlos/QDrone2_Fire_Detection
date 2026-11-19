#include "ei_detection_selector.h"
#include <cmath>
namespace ei_bridge {
void Selector::consider(const Detection &d) {
 if(d.model_class!=0 || !std::isfinite(d.confidence) ||
    d.confidence<=0 || d.confidence>1 || d.confidence<options_.min_confidence) return;
 if(!found_ || d.confidence>best_.confidence ||
   (d.confidence==best_.confidence && d.index<best_.index)) { best_=d; found_=true; }
}
void Selector::finish(ei_result &r) const {
 r.class_index=-1; r.status=EI_NO_DETECTION;
 if(!found_) return;
 r.status=EI_SUCCESS; r.detected=1; r.class_index=best_.model_class;
 r.object_class=0; // compatibility output; detected carries presence/absence
 r.confidence=100.0f*best_.confidence; r.selected_detection=uint32_t(best_.index);
 r.x=best_.x; r.y=best_.y; r.width=best_.w; r.height=best_.h;
}
}
