#ifndef JUSTFLOAT_H
#define JUSTFLOAT_H

#include <stdint.h>
#include "com_interface.h"

static inline void justfloat(const float* input, uint8_t length)
{
    float buffer[length + 1];

    for (uint8_t i = 0; i < length; ++i)
    {
        buffer[i] = input[i];
    }

    buffer[length] = (union { uint32_t u; float f; }){ .u = 0x7F800000 }.f;

    Com_SCISendEnqueue(buffer, length + 1);
}

#endif  // JUSTFLOAT_H
