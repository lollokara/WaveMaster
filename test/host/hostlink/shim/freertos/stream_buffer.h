#ifndef SHIM_STREAM_H_
#define SHIM_STREAM_H_
#include <stddef.h>
#include <string.h>
#include "FreeRTOS.h"
typedef struct { uint8_t d[8192]; size_t n; size_t cap; } shim_sb;
typedef shim_sb *StreamBufferHandle_t;
static inline StreamBufferHandle_t xStreamBufferCreate(size_t cap, size_t trig)
{
    static shim_sb sb;
    (void)trig;
    sb.cap = cap;
    sb.n = 0;
    return &sb;
}
static inline size_t xStreamBufferSend(StreamBufferHandle_t h, const void *p, size_t len, TickType_t w)
{
    size_t room = h->cap - h->n, k = len < room ? len : room;
    (void)w;
    memcpy(h->d + h->n, p, k);
    h->n += k;
    return k;
}
static inline size_t xStreamBufferReceive(StreamBufferHandle_t h, void *p, size_t max, TickType_t w)
{
    size_t k = h->n < max ? h->n : max;
    (void)w;
    memcpy(p, h->d, k);
    memmove(h->d, h->d + k, h->n - k);
    h->n -= k;
    return k;
}
#endif
