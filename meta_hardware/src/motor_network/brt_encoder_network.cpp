#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/can_driver/can_exceptions.hpp"
#include "meta_hardware/motor_driver/brt_encoder_driver.hpp"
#include "meta_hardware/motor_network/brt_encoder_network.hpp"

namespace meta_hardware {
using std::string;
using std::unordered_map;
using std::vector;


// BrtEncoderNetwork 类用于管理多个编码器的网络通信。
BrtEncoderNetwork::BrtEncoderNetwork(const string &can_network_name,
                               const vector<unordered_map<string, string>> &joint_params){
    // 用于存储 CAN 过滤器
    std::vector<can_filter> filters;

    // 初始化每隔编码器 

    for (const auto &joint_param : joint_params) {


        // 先定义两个变量，取自参数，然后建立映射
        // 取编码器 ID，并转换为 uint32_t 类型
        uint32_t brt_encoder_id = std::stoi(joint_param.at("encoder_id"));
        //  创建一个 BrtEncoder 对象，并使用 joint_param 进行初始化。
        auto brt_encoder = std::make_shared<BrtEncoder>(joint_param);
        // 将编码器 ID 和 BrtEncoder 对象存储在 encoder_id2encoder_ 映射中。
        encoder_id2encoder_[brt_encoder_id] = brt_encoder;
        // 将 BrtEncoder 对象添加到 brt_encoders_ 向量中

        // 这个就是添加进去的意思，brt_encoders_[0] = brt_encoder;
        brt_encoders_.emplace_back(brt_encoder);

    }

    filters.push_back({.can_id = 0x001, .can_mask = CAN_EFF_MASK});

    // Initialize CAN driver
    can_driver_ = std::make_unique<CanDriver>(can_network_name, false, filters);

    // Start RX thread
    rx_thread_ =
        std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

BrtEncoderNetwork::~BrtEncoderNetwork() = default;

double BrtEncoderNetwork::read(size_t joint_id) const {
    return brt_encoders_[joint_id]->get_encoder_feedback();
}

void BrtEncoderNetwork::rx_loop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        try {
            can_frame can_msg = can_driver_->read(2000);

            uint8_t encoder_id = can_msg.data[1];
            encoder_id2encoder_.at(encoder_id)->set_encoder_feedback(can_msg);
        } catch (CanIOTimedOutException & /*e*/) {
            std::cerr << "Timed out waiting for BRITTER encoder feedback." << std::endl;
        } catch (CanIOException &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

} // namespace meta_hardware
