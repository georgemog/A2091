#pragma once
#include <stdint.h>
void fpga_spi_fast_block_write(const uint16_t *buf, uint32_t length);
void fpga_spi_fast_block_read(uint16_t *buf, uint32_t length);
void fpga_spi_fast_block_write_be(const uint16_t *buf, uint32_t length);
void fpga_spi_fast_block_read_be(uint16_t *buf, uint32_t length);
