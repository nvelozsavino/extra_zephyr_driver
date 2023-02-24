//
// Created by nico on 22/12/21.
//

#pragma once

#include <stdint.h>

uint32_t calc_crc32(const volatile uint8_t *bytes, uint32_t size, uint32_t seed, uint32_t crc);
uint16_t calc_crc16(const volatile uint8_t *bytes, uint32_t size, uint16_t seed, uint16_t crc);
uint8_t calc_crc8(const volatile uint8_t *bytes, uint32_t size, uint8_t seed, uint8_t crc);

void buffer_append_uint16(uint16_t source, uint8_t *dst);
void buffer_append_int16(int16_t source, uint8_t *dst);

uint16_t buffer_extract_uint16(uint8_t const *src);
int16_t buffer_extract_int16(uint8_t const *src);

void buffer_append_uint32(uint32_t source, uint8_t *dst);
void buffer_append_int32(int32_t source, uint8_t *dst);

uint32_t buffer_extract_uint32(uint8_t const *src);
int32_t buffer_extract_int32(uint8_t const *src);

void buffer_append_uint16_r(uint16_t source, uint8_t *dst);
void buffer_append_int16_r(int16_t source, uint8_t *dst);

uint16_t buffer_extract_uint16_r(uint8_t const *src);
int16_t buffer_extract_int16_r(uint8_t const *src);

void buffer_append_uint32_r(uint32_t source, uint8_t *dst);
void buffer_append_int32_r(int32_t source, uint8_t *dst);

uint32_t buffer_extract_uint32_r(uint8_t const *src);
int32_t buffer_extract_int32_r(uint8_t const *src);

uint32_t str_to_uint(const char *text, uint8_t size);
