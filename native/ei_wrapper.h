#ifndef EI_WRAPPER_H
#define EI_WRAPPER_H
#include <stdint.h>
#ifdef _WIN32
# ifdef EI_WRAPPER_BUILD
#  define EI_API __declspec(dllexport)
# else
#  define EI_API __declspec(dllimport)
# endif
#else
# define EI_API
#endif
#ifdef __cplusplus
extern "C" {
#endif
#define EI_ABI_VERSION 1u
#define EI_LABEL_CAPACITY 128u
enum { EI_READY=0, EI_SUCCESS=1, EI_NO_DETECTION=2,
 EI_INVALID_IMAGE=-1, EI_NOT_INITIALIZED=-2, EI_INFERENCE_FAILED=-3,
 EI_NATIVE_FAILURE=-4, EI_UNSUPPORTED_MODEL=-5, EI_BUSY=-6,
 EI_INVALID_CONFIG=-7, EI_TOO_MANY_DETECTIONS=-8, EI_TIMEOUT=-9 };
typedef struct {
 uint32_t abi_version, width, height, channels, class_count, last_layer;
 float exported_threshold;
 char labels[2][EI_LABEL_CAPACITY]; /* labels[0] is sole class; slot 1 reserved for ABI compatibility */
} ei_model_info;
typedef struct {
 float min_confidence; /* fraction [0,1]; cannot recover boxes discarded by SDK */
 uint32_t false_class_index, max_detections, debug_log; /* false_class_index must be 0 */
 double timeout_ms; /* 0 disables; reports deadline overrun AFTER blocking call */
} ei_options;
typedef struct {
 int32_t status, class_index; /* model index; -1 means no valid detection */
 uint32_t detected, object_class, detection_count, selected_detection; /* object_class always 0; use detected */
 float confidence; /* percent */
 uint32_t x, y, width, height; /* model coordinates; FOMO regions are not full object boxes */
 uint64_t result_id;
 double elapsed_ms;
} ei_result;
EI_API int32_t ei_initialize(const ei_options *options);
EI_API int32_t ei_get_model_info(ei_model_info *info, uint32_t capacity_bytes);
/* Caller owns all buffers. No input pointer survives this synchronous call.
   Input is MATLAB column-major uint8; exactly h*w*model_info.channels bytes.
   Grayscale models take H x W (1 channel); RGB models take H x W x 3. */
EI_API int32_t ei_infer_image(const uint8_t *rgb, uint64_t bytes, uint32_t h, uint32_t w, ei_result *result, uint32_t capacity_bytes);
EI_API int32_t ei_get_result(ei_result *result, uint32_t capacity_bytes);
EI_API int32_t ei_reset(void);
EI_API int32_t ei_terminate(void);
/* Copies NUL-terminated text. Error text is per calling thread. */
EI_API int32_t ei_get_last_error(char *text, uint32_t capacity);
/* Simulation-only scalar bridge; the same DLL and inference engine as MATLAB. */
EI_API int32_t ei_sim_start(float threshold, uint32_t false_index, uint32_t max_boxes, uint32_t debug_log, double timeout_ms);
EI_API void ei_sim_stop(void);
EI_API int32_t ei_sim_infer(const uint8_t *rgb, uint64_t bytes, uint32_t h, uint32_t w, float *confidence, uint8_t *object_class, uint8_t *detected, uint64_t *result_id);
#ifdef __cplusplus
}
#endif
#endif
