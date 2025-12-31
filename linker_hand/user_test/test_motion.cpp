/**
 * @file test_motion.cpp
 * @brief LinkerHand SDK 运动测试
 *
 * 独立测试模块：测试关节运动功能
 * - 拇指微动测试
 * - 速度设置测试
 *
 * @copyright Copyright (c) 2025 灵心巧手（北京）科技有限公司
 */

#include "test_common.h"

class MotionTest : public TestBase {
public:
    int run(LINKER_HAND model, HAND_TYPE handType) {
        model_ = model;
        handType_ = handType;

        printHeader("运动测试");

        // 连接设备
        if (!connectDevice(model, handType)) {
            printSummary();
            return 1;
        }

        printSectionHeader("运动测试 (Motion Test)");

        bool allPassed = true;

        printWarning("即将进行小幅度运动测试，请确保灵巧手周围无障碍物");
        printInfo("等待 2 秒...");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        try {
            // 读取当前位置作为基准
            auto currentState = hand_->getState();
            if (currentState.empty()) {
                printTestResult("运动测试", false, "无法读取当前位置");
                printSummary();
                return 1;
            }

            // 备份当前位置
            std::vector<uint8_t> originalState = currentState;

            // 小幅度移动拇指 (第一个关节)
            printInfo("测试拇指微动...");

            std::vector<uint8_t> testPose = currentState;
            // 微调第一个关节 (+/- 10)
            int delta = (testPose[0] > 128) ? -10 : 10;
            testPose[0] = static_cast<uint8_t>(std::max(0, std::min(255, static_cast<int>(testPose[0]) + delta)));

            hand_->fingerMove(testPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 读取新位置
            auto newState = hand_->getState();

            // 恢复原位置
            hand_->fingerMove(originalState);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 验证运动是否发生
            bool motionDetected = false;
            for (size_t i = 0; i < std::min(currentState.size(), newState.size()); ++i) {
                if (std::abs(static_cast<int>(newState[i]) - static_cast<int>(currentState[i])) > 2) {
                    motionDetected = true;
                    break;
                }
            }

            if (motionDetected) {
                printTestResult("拇指微动测试", true, "运动响应正常");
            } else {
                printTestResult("拇指微动测试", false, "未检测到运动");
                allPassed = false;
            }

            // 速度设置测试
            std::vector<uint8_t> speedTest = {100, 100, 100, 100, 100};
            hand_->setSpeed(speedTest);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            printTestResult("速度设置测试", true);

        } catch (const std::exception& e) {
            printTestResult("运动测试", false, e.what());
            allPassed = false;
        }

        printSummary();
        return getResult();
    }
};

int main(int argc, char* argv[]) {
    LINKER_HAND model;
    HAND_TYPE handType;

    if (!parseArgs(argc, argv, model, handType, "运动测试")) {
        return 0;
    }

    MotionTest test;
    return test.run(model, handType);
}

