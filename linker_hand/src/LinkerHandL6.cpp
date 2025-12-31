#include "LinkerHandL6.h"

namespace LinkerHandL6
{
LinkerHand::LinkerHand(uint32_t handId, const std::string &canChannel, int baudrate)
    : handId(handId), running(true)
{
    bus = Communication::CanBusFactory::createCanBus(handId, canChannel, baudrate, LINKER_HAND::L6);

    // 初始化触觉矩阵数据：5个手指，每个12x6矩阵
    touch_mats.assign(5, std::vector<std::vector<uint8_t>>(12, std::vector<uint8_t>(6, 0)));

    // 初始化数据向量
    joint_position.resize(6, 0);
    joint_speeds.resize(6, 0);
    joint_torques.resize(6, 0);
    joint_accelerations.resize(6, 0);
    motor_temperature.resize(6, 0);
    error_codes.resize(6, 0);

    receiveThread = std::thread(&LinkerHand::receiveResponse, this);
    
    // 初始化时请求设备信息
    requestDeviceInfo();
    getTemperature();
    setSpeed(std::vector<uint8_t>(6, 64));
    getCurrentStatus();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

LinkerHand::~LinkerHand()
{
    running = false;
    if (receiveThread.joinable())
    {
        receiveThread.join();
    }
}

void LinkerHand::requestDeviceInfo()
{
    // 请求版本信息
    bus->send({DEVICE_SOFTWARE_VERSION}, handId);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 请求序列号
    bus->send({DEVICE_SERIAL_NUMBER}, handId);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 请求硬件版本
    bus->send({DEVICE_HARDWARE_VERSION}, handId);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 请求机械版本
    bus->send({DEVICE_MECHANICAL_VERSION}, handId);
}

void LinkerHand::setJointPositions(const std::vector<uint8_t> &jointAngles)
{
    if (jointAngles.size() == 6) {
        joint_position = jointAngles;
        std::vector<uint8_t> result(jointAngles.begin(), jointAngles.end());
        result.insert(result.begin(), JOINT_POSITION);
        bus->send(result, handId);
    } else {
        std::cout << "Joint position size is not 6" << std::endl;
    }
}

void LinkerHand::setSpeed(const std::vector<uint8_t> &speed)
{
    if (speed.size() == 6) {
        std::vector<uint8_t> result(speed.begin(), speed.end());
        result.insert(result.begin(), JOINT_SPEED);
        bus->send(result, handId);
    } else {
        std::cout << "Speed size is not 6" << std::endl;
    }
}

void LinkerHand::setTorque(const std::vector<uint8_t> &torque)
{
    if (torque.size() == 6) {
        std::vector<uint8_t> result(torque.begin(), torque.end());
        result.insert(result.begin(), TORQUE_LIMIT);
        bus->send(result, handId);
    } else {
        std::cout << "Torque size is not 6" << std::endl;
    }
}

void LinkerHand::setAcceleration(const std::vector<uint8_t> &acceleration)
{
    if (acceleration.size() == 6) {
        std::vector<uint8_t> result(acceleration.begin(), acceleration.end());
        result.insert(result.begin(), JOINT_ACCELERATION);
        bus->send(result, handId);
    } else {
        std::cout << "Acceleration size is not 6" << std::endl;
    }
}


void LinkerHand::reHome()
{
    std::vector<uint8_t> command = {RE_HOME_COMMAND, 0x01}; // 重新调零命令
    bus->send(command, handId);
}

void LinkerHand::factoryReset()
{
    std::vector<uint8_t> command = {FACTORY_RESET, 0x01}; // 恢复出厂设置
    bus->send(command, handId);
}

void LinkerHand::saveParameters()
{
    std::vector<uint8_t> command = {SAVE_PARAMETERS, 0x01}; // 保存参数
    bus->send(command, handId);
}

void LinkerHand::setJointPositionArc(const std::vector<double> &jointAngles)
{
    if (jointAngles.size() == 6) {
        if (handId == HAND_TYPE::LEFT) {
            setJointPositions(arc_to_range(6, "left", jointAngles));
        } else if (handId == HAND_TYPE::RIGHT) {
            setJointPositions(arc_to_range(6, "right", jointAngles));
        }
    } else {
        std::cout << "Joint position size is not 6" << std::endl;
    }
}

std::vector<uint8_t> LinkerHand::getCurrentStatus()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    bus->send({JOINT_POSITION}, handId);
    return joint_position;
}

std::vector<double> LinkerHand::getCurrentStatusArc()
{
    std::vector<uint8_t> current_status = getCurrentStatus();
    if (handId == HAND_TYPE::LEFT) {
        return range_to_arc(6, "left", current_status);
    } else if (handId == HAND_TYPE::RIGHT) {
        return range_to_arc(6, "right", current_status);
    }
    return {};
}

std::vector<uint8_t> LinkerHand::getSpeed()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // bus->send({JOINT_SPEED}, handId);
    return joint_speeds;
}

std::vector<uint8_t> LinkerHand::getTorque()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 注意：这里获取的是设置的转矩限制，不是实际转矩
    // 如果需要实际转矩，可能需要其他指令或计算
    bus->send({TORQUE_LIMIT}, handId);
    return joint_torques;
}

std::vector<uint8_t> LinkerHand::getAcceleration()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    bus->send({JOINT_ACCELERATION}, handId);
    return joint_accelerations;
}

void LinkerHand::clearFaultCode(const std::vector<uint8_t> &code)
{
    if (code.size() == 6) {
        std::vector<uint8_t> result(code.begin(), code.end());
        result.insert(result.begin(), CLEAR_FAULT_CODE);
        bus->send(result, handId);
    } else {
        std::cout << "Fault code size is not 6" << std::endl;
    }
}

std::vector<uint8_t> LinkerHand::getFaultCode()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    bus->send({MOTOR_ERROR_CODE}, handId);
    return error_codes;
}

std::string LinkerHand::getVersion()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::stringstream ss;
    
    if (software_version.size() >= 4) {
        ss << "Software Version: " 
           << (int)software_version[1] << "." 
           << (int)software_version[2] << "." 
           << (int)software_version[3] << std::endl;
    }
    
    if (hardware_version.size() >= 4) {
        ss << "Hardware Version: " 
           << (int)hardware_version[1] << "." 
           << (int)hardware_version[2] << "." 
           << (int)hardware_version[3] << std::endl;
    }
    
    if (mechanical_version.size() >= 4) {
        ss << "Mechanical Version: " 
           << (int)mechanical_version[1] << "." 
           << (int)mechanical_version[2] << "." 
           << (int)mechanical_version[3] << std::endl;
    }

    return ss.str();
}

std::string LinkerHand::getSerialNumber()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (device_serial.empty()) {
        return "Unknown";
    }
    
    std::string serial;
    for (size_t i = 1; i < device_serial.size() && i < 24; i++) {
        if (device_serial[i] != 0) {
            serial += static_cast<char>(device_serial[i]);
        }
    }
    return serial;
}

std::string LinkerHand::getHardwareVersion()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (hardware_version.size() < 4) {
        return "Unknown";
    }
    
    std::stringstream ss;
    ss << (int)hardware_version[1] << "." 
       << (int)hardware_version[2] << "." 
       << (int)hardware_version[3];
    return ss.str();
}

std::string LinkerHand::getMechanicalVersion()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (mechanical_version.size() < 4) {
        return "Unknown";
    }
    
    std::stringstream ss;
    ss << (int)mechanical_version[1] << "." 
       << (int)mechanical_version[2] << "." 
       << (int)mechanical_version[3];
    return ss.str();
}

std::vector<uint8_t> LinkerHand::getTemperature()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    bus->send({MOTOR_TEMPERATURE}, handId);
    return motor_temperature;
}

std::vector<std::vector<std::vector<uint8_t>>> LinkerHand::getForce() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (sensor_type == TOUCH_TYPE_MATRIX) {
        // 请求所有手指的触觉数据
        bus->send({THUMB_TOUCH, TOUCH_PAGE_REQ}, handId);
        bus->send({INDEX_TOUCH, TOUCH_PAGE_REQ}, handId);
        bus->send({MIDDLE_TOUCH, TOUCH_PAGE_REQ}, handId);
        bus->send({RING_TOUCH, TOUCH_PAGE_REQ}, handId);
        bus->send({LITTLE_TOUCH, TOUCH_PAGE_REQ}, handId);
    }
    return touch_mats;
}

void LinkerHand::parseSerialNumber(const std::vector<uint8_t>& data)
{
    if (data.size() >= 2) {
        // 序列号数据需要多帧组合，这里简化处理
        device_serial = data;
    }
}

void LinkerHand::receiveResponse()
{
    while (running)
    {
        try
        {
            auto frame = bus->recv(handId);
            std::vector<uint8_t> data(frame.data, frame.data + frame.can_dlc);
            if (data.size() <= 0) continue;
            
            if (RECV_DEBUG) {
                std::cout << "# L6-Recv " << getCurrentTime() << " | can_id:" << std::hex << frame.can_id << std::dec << " can_dlc:" << (int)frame.can_dlc << " data:";
                for (auto &can : data) std::cout << std::hex << (int)can << std::dec << " ";
                std::cout << std::endl;
            }

            uint8_t frame_property = data[0];
            std::vector<uint8_t> payload(data.begin() + 1, data.end()); // 去掉指令码

            std::lock_guard<std::mutex> lock(data_mutex_);

            // 处理触觉传感器数据
            if (frame_property >= THUMB_TOUCH && frame_property <= LITTLE_TOUCH) 
            {
                if (data.size() == 3) {
                    // 触觉传感器类型响应
                    if (sensor_type != TOUCH_TYPE_MATRIX) {
                        sensor_type = TOUCH_TYPE_MATRIX;
                        continue;
                    }
                }
                if (sensor_type == TOUCH_TYPE_MATRIX && data.size() == 8) {
                    uint8_t page_index = (data[1] >> 4) & 0x0F;
                    std::vector<uint8_t> sensor_data(data.begin() + 2, data.end());

                    const std::size_t finger_index = (frame_property & 0x0F) - 1; // B1->0, B2->1, etc.
                    if (finger_index < touch_mats.size() && page_index < touch_mats[finger_index].size() && 
                        sensor_data.size() <= touch_mats[finger_index][page_index].size()) {
                        std::memcpy(touch_mats[finger_index][page_index].data(), sensor_data.data(), sensor_data.size());
                    }
                }
                continue;
            }

            // 处理其他指令响应
            switch(frame_property) {
                case JOINT_POSITION:
                    if (payload.size() >= 6) {
                        joint_position.assign(payload.begin(), payload.begin() + 6);
                    }
                    break;
                    
                case JOINT_SPEED:
                    if (payload.size() >= 6) {
                        joint_speeds.assign(payload.begin(), payload.begin() + 6);
                    }
                    break;
                    
                case TORQUE_LIMIT:
                    if (payload.size() >= 6) {
                        joint_torques.assign(payload.begin(), payload.begin() + 6);
                    }
                    break;
                    
                case JOINT_ACCELERATION:
                    if (payload.size() >= 6) {
                        joint_accelerations.assign(payload.begin(), payload.begin() + 6);
                    }
                    break;
                    
                case MOTOR_TEMPERATURE:
                    if (payload.size() >= 6) {
                        motor_temperature.assign(payload.begin(), payload.begin() + 6);
                    }
                    break;
                    
                case MOTOR_ERROR_CODE:
                    if (payload.size() >= 6) {
                        error_codes.assign(payload.begin(), payload.begin() + 6);
                    }
                    break;
                    
                case DEVICE_SERIAL_NUMBER:
                    parseSerialNumber(data);
                    break;
                    
                case DEVICE_HARDWARE_VERSION:
                    if (data.size() >= 4) {
                        hardware_version = data;
                    }
                    break;
                    
                case DEVICE_SOFTWARE_VERSION:
                    if (data.size() >= 4) {
                        software_version = data;
                    }
                    break;
                    
                case DEVICE_MECHANICAL_VERSION:
                    if (data.size() >= 4) {
                        mechanical_version = data;
                    }
                    break;
                    
                default:
                    if (RECV_DEBUG) std::cout << "L6 Unknown data type: " << std::hex << (int)frame_property << std::dec << std::endl;
                    continue;
            }
        }
        catch (const std::runtime_error &e)
        {
            if (RECV_DEBUG) std::cerr << "Error receiving data: " << e.what() << std::endl;
        }
    }
}
}
