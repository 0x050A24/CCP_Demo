#pragma once

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    bool     is_ext;
    bool     is_rtr;
} can_frame_t;

