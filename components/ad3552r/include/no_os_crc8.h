#ifndef NO_OS_CRC8_H_
#define NO_OS_CRC8_H_

#include <stdint.h>
#include <stddef.h>

#define NO_OS_CRC8_TABLE_SIZE 256

void no_os_crc8_populate_msb(uint8_t table[NO_OS_CRC8_TABLE_SIZE], uint8_t poly);
uint8_t no_os_crc8(const uint8_t table[NO_OS_CRC8_TABLE_SIZE], const uint8_t *data,
                    uint32_t len, uint8_t seed);

#endif /* NO_OS_CRC8_H_ */
