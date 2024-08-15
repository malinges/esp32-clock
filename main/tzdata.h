#pragma once

#include <stddef.h>

typedef struct {
    const char *name;
    const char *tz;
} timezone_t;

extern const timezone_t timezones[];
extern const size_t timezones_len;
