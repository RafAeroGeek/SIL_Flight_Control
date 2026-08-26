#ifndef DYNAMIC_MODELS_H
#define DYNAMIC_MODELS_H

#include "dynamics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DYN_MODEL_SS_V1 = 0,
    DYN_MODEL_SS_V2,
    // agrega más...
    DYN_MODEL_COUNT
} DynamicModelId;

// Devuelve un Params por valor (copia). C11 lo soporta bien.
Params get_dynamic_model(DynamicModelId id);

#ifdef __cplusplus
}
#endif

#endif
