/**
 * @file test_environment.cpp
 * @brief LinkerHand SDK 环境检测测试
 *
 * 独立测试模块：检查系统环境和通信接口
 * - Linux: 检测 CAN 接口 (can0/can1)
 * - Windows: 检测 PCAN 驱动
 *
 * @copyright Copyright (c) 2025 灵心巧手（北京）科技有限公司
 */

#include "test_common.h"

class EnvironmentTest : public TestBase {
public:
    int run(LINKER_HAND model, HAND_TYPE handType) {
        model_ = model;
        handType_ = handType;

        printHeader("环境检测测试");

        std::cout << "测试配置:\n";
        std::cout << "  ├── 型号: " << getModelName(model) << "\n";
        std::cout << "  └── 手部: " << getHandTypeName(handType) << "\n";

        printSectionHeader("环境检测 (Environment Check)");

        bool allPassed = true;

        // 系统信息
#ifdef PLATFORM_LINUX
        printInfo("系统: Linux/Ubuntu");
#else
        printInfo("系统: Windows");
#endif

        // 通信接口检测
#ifdef PLATFORM_LINUX
        bool can0 = checkCanInterface("can0");
        bool can1 = checkCanInterface("can1");

        std::string canStatus = "CAN 接口: ";
        canStatus += can0 ? "can0 ✓" : "can0 ✗";
        canStatus += "  ";
        canStatus += can1 ? "can1 ✓" : "can1 ✗";

        if (can0 || can1) {
            printTestResult("CAN 接口检测", true, canStatus);
        } else {
            printTestResult("CAN 接口检测", false, "未找到可用的 CAN 接口");
            printError("请确保 CAN 接口已启用: sudo ip link set can0 up type can bitrate 1000000");
            allPassed = false;
        }

        // 权限检查
        bool hasPermission = (getuid() == 0) || (access("/dev/net/tun", R_OK) == 0);
        printTestResult("权限检查", true, hasPermission ? "root 或有足够权限" : "普通用户");

#else  // Windows
        bool pcanOk = checkPcanDriver();
        printTestResult("PCAN 驱动检测", pcanOk,
            pcanOk ? "PCANBasic.dll 已加载" : "未找到 PCAN 驱动");

        if (!pcanOk) {
            printError("请确保 PCANBasic.dll 已复制到 C:\\Windows\\System32");
            allPassed = false;
        }

        // 检查 PCAN 设备
        printInfo("PCAN 设备: 请确保 PCAN-USB 适配器已连接");
#endif

        printSummary();
        return getResult();
    }
};

int main(int argc, char* argv[]) {
    LINKER_HAND model;
    HAND_TYPE handType;

    if (!parseArgs(argc, argv, model, handType, "环境检测测试")) {
        return 0;
    }

    EnvironmentTest test;
    return test.run(model, handType);
}

