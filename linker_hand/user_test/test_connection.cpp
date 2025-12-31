/**
 * @file test_connection.cpp
 * @brief LinkerHand SDK 硬件连接测试
 *
 * 独立测试模块：检测灵巧手设备连接
 * - 创建 API 实例
 * - 验证设备通信
 * - 获取固件版本
 *
 * @copyright Copyright (c) 2025 灵心巧手（北京）科技有限公司
 */

#include "test_common.h"

class ConnectionTest : public TestBase {
public:
    int run(LINKER_HAND model, HAND_TYPE handType) {
        model_ = model;
        handType_ = handType;

        printHeader("硬件连接测试");

        std::cout << "测试配置:\n";
        std::cout << "  ├── 型号: " << getModelName(model) << "\n";
        std::cout << "  ├── 手部: " << getHandTypeName(handType) << "\n";
        std::cout << "  └── 关节数: " << getJointCount(model) << "\n";

        printSectionHeader("硬件连接测试 (Hardware Connection)");

        printInfo("正在连接灵巧手...");

        try {
            // 创建 API 实例
            hand_ = new LinkerHandApi(model_, handType_);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 获取版本信息
            std::string version = hand_->getVersion();

            if (!version.empty() && version != "0.0.0.0") {
                printTestResult("设备连接", true);
                printSuccess("型号: " + getModelName(model_));
                printSuccess("手部: " + getHandTypeName(handType_));
                printSuccess("固件版本: " + version);
            } else {
                printTestResult("设备连接", false, "无法获取版本信息");
                printError("请检查硬件连接和通信线路");
            }

        } catch (const std::exception& e) {
            printTestResult("设备连接", false, e.what());
            printError("\n硬件连接失败，请检查:");
            printError("  1. 灵巧手电源是否开启");
            printError("  2. CAN/PCAN 线缆是否正确连接");
            printError("  3. 型号和手部类型是否正确");
        }

        printSummary();
        return getResult();
    }
};

int main(int argc, char* argv[]) {
    LINKER_HAND model;
    HAND_TYPE handType;

    if (!parseArgs(argc, argv, model, handType, "硬件连接测试")) {
        return 0;
    }

    ConnectionTest test;
    return test.run(model, handType);
}

