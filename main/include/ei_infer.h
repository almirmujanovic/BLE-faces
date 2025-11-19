#ifndef EI_INFER_H
#define EI_INFER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize Edge Impulse (allocate buffers)
bool ei_infer_init(void);

// Cleanup
void ei_infer_deinit(void);

// Run inference on current camera frame
const char* ei_infer_gesture(float *confidence_out);

#ifdef __cplusplus
}
#endif

#endif // EI_INFER_H