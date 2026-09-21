#ifndef NO_OS_UTIL_H_
#define NO_OS_UTIL_H_

#include <stdint.h>
#include <string.h>

#define NO_OS_BIT(x)            (1U << (x))
#define NO_OS_GENMASK(h, l) \
    ((0xFFFFFFFFU << (l)) & (0xFFFFFFFFU >> (31U - (h))))
#define NO_OS_ARRAY_SIZE(x)     (sizeof(x) / sizeof((x)[0]))
#define NO_OS_DIV_ROUND_CLOSEST(x, d) (((x) + (d) / 2) / (d))
#define NO_OS_IS_ERR_VALUE(x)  ((x) < 0)

static inline uint32_t no_os_field_get(uint32_t mask, uint32_t val)
{
    if (!mask)
        return 0;
    return (val & mask) >> __builtin_ctz(mask);
}

static inline uint32_t no_os_field_prep(uint32_t mask, uint32_t val)
{
    if (!mask)
        return 0;
    return (val << __builtin_ctz(mask)) & mask;
}

static inline void no_os_put_unaligned_be16(uint16_t val, uint8_t *buf)
{
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
}

static inline uint16_t no_os_get_unaligned_be16(const uint8_t *buf)
{
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static inline int32_t no_os_find_first_set_bit(uint32_t mask)
{
    if (!mask)
        return 32;
    return __builtin_ctz(mask);
}

#endif /* NO_OS_UTIL_H_ */
