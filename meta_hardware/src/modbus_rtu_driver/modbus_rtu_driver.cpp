
#include "meta_hardware/modbus_rtu_driver/modbus_rtu_driver.hpp"
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>
#include <chrono>
#include <boost/crc.hpp>
#include <sys/types.h>
#include <iostream>
#include <thread>
#include <vector>

ModbusRtuDriver::ModbusRtuDriver(std::unordered_map<std::string, std::string> serial_params): 
    // std::unordered_map<std::string, std::string> 类型的变量，表示一个键值对的集合
    // std::unordered_map<std::string, std::string> serial_params = {
    //     {"baud_rate", "9600"},
    //     {"parity", "odd"},
    //     {"stop_bits", "1.0"},
    //     {"flow_control", "none"},
    //     {"device_name", "/dev/ttyUSB0"}
    // };


    owned_ctx_{new IoContext(2)},
    // 初始化 serial_driver_，创建一个新的 SerialDriver 对象，并传递 owned_ctx_
    serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}



    {
    command_.resize(8);

    read_params(serial_params);
    // std::make_unique 是 C++14 引入的模板函数，用于创建一个动态分配的对象，并将其所有权转移给一个 std::unique_pt
    // 创建一个新的 SerialPortConfig 对象，并使用读取的参数进行初始化
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>
    (baud_rate_, flow_control_, parity_, stop_bits_);
    
    try {
        // 初始化串行端口。
        serial_driver_->init_port(device_name_, *device_config_);
        // 打开串行端口
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
            // 创建并启动接收线程，执行 rx_tx_loop 方法
            receive_thread_ = std::make_unique<std::jthread>([this](std::stop_token s) { rx_tx_loop(s); });
        }
    } catch (const std::exception & ex) {
        std::cerr << "Error creating serial port: " << device_name_ << " - " << ex.what() << std::endl;
        throw ex;
    }

}


// 读取串口参数
void ModbusRtuDriver::read_params(std::unordered_map<std::string, std::string> serial_params){

    // at 是 std::unordered_map 提供的一个成员函数，用于通过键访问对应的值。
    // 如果键 "parity" 存在于 serial_params 中，at("parity") 会返回该键对应的值。
    // 如果键 "parity" 不存在，at 会抛出一个 std::out_of_range 异常



    // 读取并转换波特率
    baud_rate_ = std::stoi(serial_params.at("baud_rate"));

    // 读取并转换流控制参数
    std::string fc = serial_params.at("flow_control");
    if (fc == "none") {
        flow_control_ = drivers::serial_driver::FlowControl::NONE;
    } else if (fc == "hardware") {
        flow_control_ = drivers::serial_driver::FlowControl::HARDWARE;
    } else if (fc == "software") {
        flow_control_ = drivers::serial_driver::FlowControl::SOFTWARE;
    } else {
        throw std::runtime_error("Unknown flow control: " + fc);
    }
    

    // 读取并转换奇偶校验参数
    std::string pt = serial_params.at("parity");
    if (pt == "none") {
        parity_ = drivers::serial_driver::Parity::NONE;
    } else if (pt == "odd") {
        parity_ = drivers::serial_driver::Parity::ODD;
    } else if (pt == "even") {
        parity_ = drivers::serial_driver::Parity::EVEN;
    } else {
        throw std::runtime_error("Unknown parity: " + pt);
    }


    // 读取并转换停止位参数
    float sb = std::stof(serial_params.at("stop_bits"));
    if (sb == 1.0) {
        stop_bits_ = drivers::serial_driver::StopBits::ONE;
    } else if (sb == 1.5) {
        stop_bits_ = drivers::serial_driver::StopBits::ONE_POINT_FIVE;
    } else if (sb == 2.0) {
        stop_bits_ = drivers::serial_driver::StopBits::TWO;
    } else {
        throw std::runtime_error("Unknown stop bits(Please Use Floating Point Notation): " + std::to_string(sb));
    }

    device_name_ = serial_params.at("device_name");
};


auto ModbusRtuDriver::awake_time(){
    using std::chrono::operator""ms;
    return std::chrono::steady_clock::now() + 10ms;
}


// 设置 Modbus RTU 命令
void ModbusRtuDriver::set_command(int addr, int func, int reg, int len){
    // 设置从站地址
    command_[0] = static_cast<uint8_t>(addr);
    // 设置功能码   
    command_[1] = static_cast<uint8_t>(func);
    // 设置寄存器地址高字节
    command_[2] = static_cast<uint8_t>(reg >> 8);
    // 设置寄存器地址低字节
    command_[3] = static_cast<uint8_t>(reg & 0xFF);
    // 设置寄存器数量高字节
    command_[4] = static_cast<uint8_t>(len >> 8);
    // 设置寄存器数量低字节
    command_[5] = static_cast<uint8_t>(len & 0xFF);


    //  计算 CRC 校验码
    boost::crc_optimal<16, 0x8005, 0xFFFF, 0, true, true> crc_16;
    crc_16.process_bytes(command_.data(), 6);
    int crc = crc_16.checksum();
    // 设置 CRC 校验码低字节
    command_[6] = static_cast<uint8_t>(crc & 0xFF);
    // 设置 CRC 校验码高字节
    command_[7] = static_cast<uint8_t>(crc >> 8);
    // 调整寄存器数据的大小
    reg_data_.resize(len);
}



// rx_tx_loop用于处理 Modbus RTU 命令的发送和接收。
void ModbusRtuDriver::rx_tx_loop(std::stop_token stop_token){
    // 计算寄存器数量
    int reg_num = static_cast<int>(command_[4] << 8 | command_[5]);
    // 计算响应的大小 response_size 是响应帧的大小，计算结果为 13。
    int response_size = reg_num * 2 + 5;
    // std::cout << "Response Size: " << response_size << std::endl;
    std::cout << "Reponse Size:" << response_size << std::endl;

    // 当未请求停止时，循环执行
    while (!stop_token.stop_requested()) {
        // 记录当前时间
        auto start_time = std::chrono::steady_clock::now(); 

        // std::cout << "Sending Command..." << std::endl;
        // 发送命令
        serial_driver_->port()->send(command_);
        

        using std::chrono::operator""ms;
        std::this_thread::sleep_for(100ms);

        std::vector<uint8_t> response(response_size);
        serial_driver_->port()->receive(response);

        if(response[0] == command_[0] && response[1] == command_[1] && response[2] == (uint8_t)reg_num*2){

            // 仅实现读取模式；写入模式未实现
            for(int i = 0; i < reg_num; i++){
                reg_data_[i] = static_cast<int>(response[3 + i * 2] << 8 | response[4 + i * 2]);
            }
            boost::crc_optimal<16, 0x8005, 0xFFFF, 0, true, true> crc_16;
            crc_16.process_bytes(response.data(), response_size - 2);
            int crc = crc_16.checksum();
            if((crc & 0xFF) != response[response_size - 2] || (crc >> 8) != response[response_size - 1]){
                std::cout << "CRC Checked Failure. " << std::endl;
            }
        }else{
            std::cout << "Feedback Format Error." << std::endl;
        }
        // for(size_t i = 0; i < response.size(); i++){
        //     std::cout << std::hex << static_cast<int>(response[i]) << " ";
        // }
        // std::cout << std::endl;

        std::this_thread::sleep_until(start_time + 250ms);
    }
}


ModbusRtuDriver::~ModbusRtuDriver(){
        // 检查 receive_thread_ 是否可以被 join，即线程是否仍在运行。
        if (receive_thread_->joinable()) {
            // 果线程可以被 join，则等待线程完成执行
            receive_thread_->join();
        }

        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }

        // 检查 owned_ctx_ 是否有效。
        if (owned_ctx_) {
            // 调用 owned_ctx_ 的 waitForExit 方法，等待上下文退
            owned_ctx_->waitForExit();
        }
    }
