/**
 * @file test_basic_function.cpp
 * @brief LinkerHand SDK 基本功能测试
 *
 * 独立测试模块：测试基本读取功能
 * - 关节位置读取
 * - 速度读取
 *
 * @copyright Copyright (c) 2025 灵心巧手（北京）科技有限公司
 */

#include "test_common.h"

class BasicFunctionTest : public TestBase {
public:
    int run(LINKER_HAND model, HAND_TYPE handType) {
        model_ = model;
        handType_ = handType;

        printHeader("基本功能测试");

        // 连接设备
        if (!connectDevice(model, handType)) {
            printSummary();
            return 1;
        }

        printSectionHeader("基本功能测试 (Basic Function)");

        bool allPassed = true;

        // 读取当前关节位置
        try {
            auto state = hand_->getState();
            int expectedJoints = getJointCount(model_);

            if (state.size() == static_cast<size_t>(expectedJoints)) {
                printTestResult("关节位置读取", true, vectorToString(state));
            } else {
                printTestResult("关节位置读取", false,
                    "期望 " + std::to_string(expectedJoints) + " 个关节，实际 " + std::to_string(state.size()));
                allPassed = false;
            }
        } catch (const std::exception& e) {
            printTestResult("关节位置读取", false, e.what());
            allPassed = false;
        }

        // 读取速度设置
        try {
            auto speed = hand_->getSpeed();
            if (!speed.empty()) {
                printTestResult("速度读取", true, vectorToString(speed));
            } else {
                printTestResult("速度读取", false, "返回空数据");
                allPassed = false;
            }
        } catch (const std::exception& e) {
            printTestResult("速度读取", false, e.what());
            allPassed = false;
        }

        printSummary();
        return getResult();
    }
};

int main(int argc, char* argv[]) {
    LINKER_HAND model;
    HAND_TYPE handType;

    if (!parseArgs(argc, argv, model, handType, "基本功能测试")) {
        return 0;
    }

    BasicFunctionTest test;
    return test.run(model, handType);
}

