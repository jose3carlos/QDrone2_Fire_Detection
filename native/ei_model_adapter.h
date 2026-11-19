#pragma once
#include "ei_wrapper.h"
#include <string>
namespace ei_bridge {
int32_t model_info(ei_model_info &info, std::string &error);
int32_t model_init(std::string &error);
void model_close();
int32_t model_infer(const uint8_t *rgb, const ei_options &options, ei_result &result, std::string &error);
}
