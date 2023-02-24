//
// Created by nico on 22/12/21.
//

#include "utils.h"

void buffer_append_uint16(uint16_t source, uint8_t *dst) {
    dst[0] = (uint8_t)(source >> 8);
    dst[1] = (uint8_t)(source);
}

void buffer_append_int16(int16_t source, uint8_t *dst) {
    buffer_append_uint16((uint16_t)source, dst);
}

uint16_t buffer_extract_uint16(uint8_t const *src) {
    uint16_t result = 0;
    result |= (uint16_t)(src[0] << 8);
    result |= (uint16_t)(src[1]);
    return result;
}

int16_t buffer_extract_int16(uint8_t const *src) {
    return (int16_t)buffer_extract_uint16(src);
}

void buffer_append_uint32(uint32_t source, uint8_t *dst) {
    dst[0] = (uint8_t)(source >> 24);
    dst[1] = (uint8_t)(source >> 16);
    dst[2] = (uint8_t)(source >> 8);
    dst[3] = (uint8_t)(source);
}

void buffer_append_int32(int32_t source, uint8_t *dst) {
    buffer_append_uint32((uint32_t)source, dst);
}

uint32_t buffer_extract_uint32(uint8_t const *src) {
    uint32_t result = 0;
    result |= (uint32_t)(src[0] << 24);
    result |= (uint32_t)(src[1] << 16);
    result |= (uint32_t)(src[2] << 8);
    result |= (uint32_t)(src[3]);
    return result;
}

int32_t buffer_extract_int32(uint8_t const *src) {
    return (int32_t)buffer_extract_uint32(src);
}

void buffer_append_uint16_r(uint16_t source, uint8_t *dst) {
    dst[1] = (uint8_t)(source >> 8);
    dst[0] = (uint8_t)(source);
}

void buffer_append_int16_r(int16_t source, uint8_t *dst) {
    buffer_append_uint16_r((uint16_t)source, dst);
}

uint16_t buffer_extract_uint16_r(uint8_t const *src) {
    uint16_t result = 0;
    result |= (uint16_t)(src[1] << 8);
    result |= (uint16_t)(src[0]);
    return result;
}

int16_t buffer_extract_int16_r(uint8_t const *src) {
    return (int16_t)buffer_extract_uint16_r(src);
}

void buffer_append_uint32_r(uint32_t source, uint8_t *dst) {
    dst[3] = (uint8_t)(source >> 24);
    dst[2] = (uint8_t)(source >> 16);
    dst[1] = (uint8_t)(source >> 8);
    dst[0] = (uint8_t)(source);
}

void buffer_append_int32_r(int32_t source, uint8_t *dst) {
    buffer_append_uint32_r((uint32_t)source, dst);
}

uint32_t buffer_extract_uint32_r(uint8_t const *src) {
    uint32_t result = 0;
    result |= (uint32_t)(src[3] << 24);
    result |= (uint32_t)(src[2] << 16);
    result |= (uint32_t)(src[1] << 8);
    result |= (uint32_t)(src[0]);
    return result;
}

int32_t buffer_extract_int32_r(uint8_t const *src) {
    return (int32_t)buffer_extract_uint32_r(src);
}

uint32_t calc_crc32(const volatile uint8_t *bytes, uint32_t size, uint32_t seed, uint32_t crc) {
    for (uint32_t j = 0; j < size; j++) {
        uint8_t b = bytes[j];
        crc ^= (uint32_t)(b << 24); /* move byte into MSB of 32bit CRC */
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x80000000) != 0) { /* test for MSB = bit 31 */
                crc = (uint32_t)((crc << 1) ^ seed);
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint16_t calc_crc16(const volatile uint8_t *bytes, uint32_t size, uint16_t seed, uint16_t crc) {
    for (uint32_t j = 0; j < size; j++) {
        uint8_t b = bytes[j];
        crc ^= (uint16_t)(b << 8); /* move byte into MSB of 16bit CRC */
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x8000) != 0) { /* test for MSB = bit 15 */
                crc = (uint16_t)((crc << 1) ^ seed);
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t calc_crc8(const volatile uint8_t *bytes, uint32_t size, uint8_t seed, uint8_t crc) {

    for (uint32_t j = 0; j < size; j++) {
        uint8_t b = bytes[j];
        crc ^= b;
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x80) != 0) { /* test for MSB = bit 15 */
                crc = (uint8_t)((crc << 1) ^ seed);
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint32_t str_to_uint(const char *text, uint8_t size) {
    uint32_t number = 0;
    uint32_t mult = 1;
    while (size > 0) {
        number += (text[size - 1] - '0') * mult;
        mult *= 10;
        size--;
    }
    return number;
}
