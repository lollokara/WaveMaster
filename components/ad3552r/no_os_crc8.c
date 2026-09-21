#include "no_os_crc8.h"

void no_os_crc8_populate_msb(uint8_t table[NO_OS_CRC8_TABLE_SIZE], uint8_t poly)
{
    int i, j;
    uint8_t crc;

    for (i = 0; i < 256; i++) {
        crc = (uint8_t)i;
        for (j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ poly);
            else
                crc = (uint8_t)(crc << 1);
        }
        table[i] = crc;
    }
}

uint8_t no_os_crc8(const uint8_t table[NO_OS_CRC8_TABLE_SIZE], const uint8_t *data,
                    uint32_t len, uint8_t seed)
{
    uint8_t crc = seed;
    uint32_t i;

    for (i = 0; i < len; i++)
        crc = table[(crc ^ data[i]) & 0xFF];

    return crc;
}
