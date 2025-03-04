#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <tuple>

#include "angles/angles.h"
#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/motor_driver/brt_encoder_driver.hpp"

namespace meta_hardware {
constexpr uint16_t MAX_RAW_POSITION = 65535;
constexpr double MAX_ABS_POSITION = 4.0 * M_PI;


// BrtEncoder 类用于处理编码器的初始化和反馈
BrtEncoder::BrtEncoder(const std::unordered_map<std::string, std::string> &motor_param) {
    encoder_model_ = motor_param.at("encoder_model");
    // 将其转换为 uint8_t 类型
    brt_encoder_id_ = static_cast<uint8_t>(std::stoi(motor_param.at("encoder_id")));

    if (encoder_model_ == "BRT38-1024") {
        resolution_ = 1024;
    } else {
        throw std::runtime_error("Unknown encoder model: " + encoder_model_);
    }
}


// 接受一个 can_frame 类型的参数 can_msg。
void BrtEncoder::set_encoder_feedback(const can_frame &can_msg) {
    uint32_t position_raw =
        can_msg.data[3] | can_msg.data[4] << 8 | can_msg.data[5] << 16;

    // 将原始位置值转换为弧度。是编码器的分辨率
    double new_position = position_raw * (2 * M_PI) / resolution_;
    // 使用 angles::shortest_angular_distance 方法计算当前位置和新位置之间的最短角度，并更新 position_ 成员变量
    position_ = position_ + angles::shortest_angular_distance(position_, new_position);
}

double BrtEncoder::get_encoder_feedback() const { return position_; }

} // namespace meta_hardware
