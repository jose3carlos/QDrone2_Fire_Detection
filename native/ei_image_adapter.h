#pragma once
#include <cstddef>
#include <cstdint>
namespace ei_bridge {
// Returns a packed 0xRRGGBB pixel at native row-major pixel offset.
uint32_t packed_pixel(const uint8_t *matlab_image, uint32_t h, uint32_t w, size_t offset, uint32_t channels=3);
}
