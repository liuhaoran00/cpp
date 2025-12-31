/**
 * @file test_sensors.cpp
 * @brief LinkerHand SDK 传感器测试
 *
 * 独立测试模块：测试传感器功能
 * - 压力传感器
 * - 温度传感器
 * - 电流传感器
 *
 * @copyright Copyright (c) 2025 灵心巧手（北京）科技有限公司
 */

#include "test_common.h"

class SensorsTest : public TestBase {
public:
    int run(LINKER_HAND model, HAND_TYPE handType) {
        model_ = model;
        handType_ = handType;

        printHeader("传感器测试");

        // 连接设备
        if (!connectDevice(model, handType)) {
            printSummary();
            return 1;
        }

        printSectionHeader("传感器测试 (Sensor Test)");

        bool allPassed = true;

        // 压力传感器
        try {
            auto force = hand_->getForce();
            if (!force.empty()) {
                int totalSensors = 0;
                for (const auto& row : force) {
                    totalSensors += row.size();
                }
                printTestResult("压力传感器", true,
                    std::to_string(force.size()) + " 组, 共 " + std::to_string(totalSensors) + " 个数据点");
            } else {
                printTestResult("压力传感器", false, "无数据返回 (可能未配置触觉传感器)");
            }
        } catch (const std::exception& e) {
            printTestResult("压力传感器", false, e.what());
        }

        // 温度传感器
        try {
            auto temp = hand_->getTemperature();
            if (!temp.empty()) {
                int avgTemp = 0;
                for (auto t : temp) avgTemp += t;
                avgTemp /= temp.size();
                printTestResult("温度传感器", true,
                    "平均温度 " + std::to_string(avgTemp) + "°C");
            } else {
                printTestResult("温度传感器", false, "无数据返回");
            }
        } catch (const std::exception& e) {
            printTestResult("温度传感器", false, e.what());
        }

        // 电流传感器
        try {
            auto current = hand_->getCurrent();
            if (!current.empty()) {
                printTestResult("电流传感器", true, vectorToString(current));
            } else {
                printTestResult("电流传感器", false, "无数据返回");
            }
        } catch (const std::exception& e) {
            printTestResult("电流传感器", false, e.what());
        }

        printSummary();
        return getResult();
    }
};

int main(int argc, char* argv[]) {
    LINKER_HAND model;
    HAND_TYPE handType;

    if (!parseArgs(argc, argv, model, handType, "传感器测试")) {
        return 0;
    }

    SensorsTest test;
    return test.run(model, handType);
}

