#ifndef EI_WRAPPER_H
#define EI_WRAPPER_H

// ESP-IDF replacements for Arduino macros (from full_gestures_inferencing.h)
#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdio.h>

// Arduino compatibility layer for ESP-IDF
#ifndef Arduino_h
#define Arduino_h
// Prevent Arduino.h inclusion - provide minimal replacements
inline void ei_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}
#endif

#ifdef __cplusplus
}
#endif

// Now safe to include EI headers
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "model-parameters/model_metadata.h"

#endif // EI_WRAPPER_H