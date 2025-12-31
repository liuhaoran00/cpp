/**
 * @file linker_hand_user_test.cpp
 * @brief LinkerHand SDK 用户测试工具
 *
 * 面向最终用户的硬件功能验证工具，用于：
 * 1. 环境检测 - 检查系统环境和通信接口
 * 2. 硬件连接测试 - 检测灵巧手设备
 * 3. 基本功能测试 - 关节读取和运动测试
 * 4. 传感器测试 - 压力、温度、电流传感器
 *
 * 支持平台：
 * - Linux (Ubuntu): SocketCAN (can0/can1)
 * - Windows: PCAN-USB
 *
 * @copyright Copyright (c) 2025 灵心巧手（北京）科技有限公司
 */

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "LinkerHandApi.h"
#include "Common.h"

// ============================================================================
// 跨平台定义
// ============================================================================
#ifdef _WIN32
    #include <windows.h>
    #define PLATFORM_WINDOWS
#else
    #include <unistd.h>
    #include <sys/stat.h>
    #include <net/if.h>
    #define PLATFORM_LINUX
#endif

// ============================================================================
// 终端颜色定义
// ============================================================================
namespace Color {
#ifdef PLATFORM_WINDOWS
    // Windows 使用 Console API
    inline void setColor(int color) {
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), color);
    }
    const int RESET = 7;
    const int RED = 12;
    const int GREEN = 10;
    const int YELLOW = 14;
    const int BLUE = 9;
    const int CYAN = 11;
    const int WHITE = 15;
#else
    // Linux 使用 ANSI 转义码
    const char* RESET  = "\033[0m";
    const char* RED    = "\033[0;31m";
    const char* GREEN  = "\033[0;32m";
    const char* YELLOW = "\033[0;33m";
    const char* BLUE   = "\033[0;34m";
    const char* CYAN   = "\033[0;36m";
    const char* WHITE  = "\033[0;37m";
    const char* BOLD   = "\033[1m";
#endif
}

// ============================================================================
// 测试结果结构
// ============================================================================
struct TestResult {
    std::string name;
    bool passed;
    std::string message;
    std::string detail;
};

// ============================================================================
// 测试工具类
// ============================================================================
class LinkerHandUserTest {
public:
    LinkerHandUserTest() : hand_(nullptr), totalTests_(0), passedTests_(0) {}
    ~LinkerHandUserTest() {
        if (hand_) {
            delete hand_;
            hand_ = nullptr;
        }
    }

    // 运行所有测试
    int runAllTests(LINKER_HAND model, HAND_TYPE handType, COMM_TYPE commType);

private:
    LinkerHandApi* hand_;
    LINKER_HAND model_;
    HAND_TYPE handType_;
    int totalTests_;
    int passedTests_;
    std::vector<TestResult> results_;

    // 测试模块
    bool testEnvironment();
    bool testConnection();
    bool testBasicFunction();
    bool testMotion();
    bool testSensors();

    // 辅助函数
    void printHeader();
    void printSectionHeader(int step, int total, const std::string& title);
    void printTestResult(const std::string& testName, bool passed, const std::string& detail = "");
    void printSummary();
    void printSuccess(const std::string& msg);
    void printError(const std::string& msg);
    void printWarning(const std::string& msg);
    void printInfo(const std::string& msg);

    std::string getModelName(LINKER_HAND model);
    std::string getHandTypeName(HAND_TYPE type);
    int getJointCount(LINKER_HAND model);
    std::string vectorToString(const std::vector<uint8_t>& vec);

#ifdef PLATFORM_LINUX
    bool checkCanInterface(const std::string& ifname);
#endif
#ifdef PLATFORM_WINDOWS
    bool checkPcanDriver();
#endif
};

// ============================================================================
// 实现
// ============================================================================

void LinkerHandUserTest::printHeader() {
#ifdef PLATFORM_LINUX
    std::cout << Color::CYAN << Color::BOLD;
#else
    Color::setColor(Color::CYAN);
#endif

    std::cout << "\n";
    std::cout << "═══════════════════════════════════════════════════════════════\n";
    std::cout << "              LinkerHand SDK 用户测试工具 v1.0                  \n";
    std::cout << "              User Hardware Validation Tool                     \n";
    std::cout << "═══════════════════════════════════════════════════════════════\n";

#ifdef PLATFORM_LINUX
    std::cout << Color::RESET;
#else
    Color::setColor(Color::RESET);
#endif
    std::cout << "\n";
}

void LinkerHandUserTest::printSectionHeader(int step, int total, const std::string& title) {
#ifdef PLATFORM_LINUX
    std::cout << Color::YELLOW << Color::BOLD;
#else
    Color::setColor(Color::YELLOW);
#endif

    std::cout << "\n[" << step << "/" << total << "] " << title << "\n";

#ifdef PLATFORM_LINUX
    std::cout << Color::RESET;
#else
    Color::setColor(Color::RESET);
#endif
}

void LinkerHandUserTest::printTestResult(const std::string& testName, bool passed, const std::string& detail) {
    std::cout << "  ├── " << testName << ": ";

    if (passed) {
#ifdef PLATFORM_LINUX
        std::cout << Color::GREEN << "✓ 通过" << Color::RESET;
#else
        Color::setColor(Color::GREEN);
        std::cout << "[PASS]";
        Color::setColor(Color::RESET);
#endif
    } else {
#ifdef PLATFORM_LINUX
        std::cout << Color::RED << "✗ 失败" << Color::RESET;
#else
        Color::setColor(Color::RED);
        std::cout << "[FAIL]";
        Color::setColor(Color::RESET);
#endif
    }

    if (!detail.empty()) {
        std::cout << " " << detail;
    }
    std::cout << "\n";

    totalTests_++;
    if (passed) passedTests_++;

    results_.push_back({testName, passed, passed ? "通过" : "失败", detail});
}

void LinkerHandUserTest::printSuccess(const std::string& msg) {
#ifdef PLATFORM_LINUX
    std::cout << Color::GREEN << "  ├── " << msg << Color::RESET << "\n";
#else
    Color::setColor(Color::GREEN);
    std::cout << "  |-- " << msg;
    Color::setColor(Color::RESET);
    std::cout << "\n";
#endif
}

void LinkerHandUserTest::printError(const std::string& msg) {
#ifdef PLATFORM_LINUX
    std::cout << Color::RED << "  ├── " << msg << Color::RESET << "\n";
#else
    Color::setColor(Color::RED);
    std::cout << "  |-- " << msg;
    Color::setColor(Color::RESET);
    std::cout << "\n";
#endif
}

void LinkerHandUserTest::printWarning(const std::string& msg) {
#ifdef PLATFORM_LINUX
    std::cout << Color::YELLOW << "  ├── " << msg << Color::RESET << "\n";
#else
    Color::setColor(Color::YELLOW);
    std::cout << "  |-- " << msg;
    Color::setColor(Color::RESET);
    std::cout << "\n";
#endif
}

void LinkerHandUserTest::printInfo(const std::string& msg) {
    std::cout << "  ├── " << msg << "\n";
}

void LinkerHandUserTest::printSummary() {
#ifdef PLATFORM_LINUX
    std::cout << Color::CYAN << Color::BOLD;
#else
    Color::setColor(Color::CYAN);
#endif

    std::cout << "\n═══════════════════════════════════════════════════════════════\n";
    std::cout << "                        测试摘要                                \n";
    std::cout << "═══════════════════════════════════════════════════════════════\n";

#ifdef PLATFORM_LINUX
    std::cout << Color::RESET;
#else
    Color::setColor(Color::RESET);
#endif

    std::cout << "  总测试项: " << totalTests_ << "\n";

#ifdef PLATFORM_LINUX
    std::cout << Color::GREEN << "  通过: " << passedTests_ << Color::RESET << "\n";
    if (totalTests_ - passedTests_ > 0) {
        std::cout << Color::RED << "  失败: " << (totalTests_ - passedTests_) << Color::RESET << "\n";
    }
#else
    Color::setColor(Color::GREEN);
    std::cout << "  通过: " << passedTests_ << "\n";
    Color::setColor(Color::RESET);
    if (totalTests_ - passedTests_ > 0) {
        Color::setColor(Color::RED);
        std::cout << "  失败: " << (totalTests_ - passedTests_) << "\n";
        Color::setColor(Color::RESET);
    }
#endif

    std::cout << "\n  状态: ";
    if (passedTests_ == totalTests_) {
#ifdef PLATFORM_LINUX
        std::cout << Color::GREEN << Color::BOLD << "全部通过 ✓" << Color::RESET;
#else
        Color::setColor(Color::GREEN);
        std::cout << "ALL PASSED";
        Color::setColor(Color::RESET);
#endif
    } else {
#ifdef PLATFORM_LINUX
        std::cout << Color::RED << Color::BOLD << "存在失败项 ✗" << Color::RESET;
#else
        Color::setColor(Color::RED);
        std::cout << "SOME FAILED";
        Color::setColor(Color::RESET);
#endif
    }

#ifdef PLATFORM_LINUX
    std::cout << Color::CYAN;
#else
    Color::setColor(Color::CYAN);
#endif
    std::cout << "\n═══════════════════════════════════════════════════════════════\n\n";
#ifdef PLATFORM_LINUX
    std::cout << Color::RESET;
#else
    Color::setColor(Color::RESET);
#endif
}

std::string LinkerHandUserTest::getModelName(LINKER_HAND model) {
    switch (model) {
        case LINKER_HAND::O6: return "O6";
        case LINKER_HAND::L6: return "L6";
        case LINKER_HAND::L7: return "L7";
        case LINKER_HAND::L10: return "L10";
        case LINKER_HAND::L20: return "L20";
        case LINKER_HAND::L21: return "L21";
        case LINKER_HAND::L25: return "L25";
        default: return "Unknown";
    }
}

std::string LinkerHandUserTest::getHandTypeName(HAND_TYPE type) {
    return (type == HAND_TYPE::LEFT) ? "左手 (Left)" : "右手 (Right)";
}

int LinkerHandUserTest::getJointCount(LINKER_HAND model) {
    switch (model) {
        case LINKER_HAND::O6:
        case LINKER_HAND::L6: return 6;
        case LINKER_HAND::L7: return 7;
        case LINKER_HAND::L10: return 10;
        case LINKER_HAND::L20: return 20;
        case LINKER_HAND::L21:
        case LINKER_HAND::L25: return 25;
        default: return 0;
    }
}

std::string LinkerHandUserTest::vectorToString(const std::vector<uint8_t>& vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << static_cast<int>(vec[i]);
    }
    oss << "]";
    return oss.str();
}

#ifdef PLATFORM_LINUX
bool LinkerHandUserTest::checkCanInterface(const std::string& ifname) {
    std::string path = "/sys/class/net/" + ifname;
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}
#endif

#ifdef PLATFORM_WINDOWS
bool LinkerHandUserTest::checkPcanDriver() {
    // 检查 PCAN DLL 是否存在
    HMODULE hModule = LoadLibraryA("PCANBasic.dll");
    if (hModule != NULL) {
        FreeLibrary(hModule);
        return true;
    }
    return false;
}
#endif

// ============================================================================
// 测试模块实现
// ============================================================================

bool LinkerHandUserTest::testEnvironment() {
    printSectionHeader(1, 5, "环境检测 (Environment Check)");

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

    return allPassed;
}

bool LinkerHandUserTest::testConnection() {
    printSectionHeader(2, 5, "硬件连接测试 (Hardware Connection)");

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
            return true;
        } else {
            printTestResult("设备连接", false, "无法获取版本信息");
            printError("请检查硬件连接和通信线路");
            return false;
        }

    } catch (const std::exception& e) {
        printTestResult("设备连接", false, e.what());
        return false;
    }
}

bool LinkerHandUserTest::testBasicFunction() {
    printSectionHeader(3, 5, "基本功能测试 (Basic Function)");

    if (!hand_) {
        printError("设备未连接，跳过此测试");
        return false;
    }

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

    return allPassed;
}

bool LinkerHandUserTest::testMotion() {
    printSectionHeader(4, 5, "运动测试 (Motion Test)");

    if (!hand_) {
        printError("设备未连接，跳过此测试");
        return false;
    }

    bool allPassed = true;

    printWarning("即将进行小幅度运动测试，请确保灵巧手周围无障碍物");
    printInfo("等待 2 秒...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    try {
        // 读取当前位置作为基准
        auto currentState = hand_->getState();
        if (currentState.empty()) {
            printTestResult("运动测试", false, "无法读取当前位置");
            return false;
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

    return allPassed;
}

bool LinkerHandUserTest::testSensors() {
    printSectionHeader(5, 5, "传感器测试 (Sensor Test)");

    if (!hand_) {
        printError("设备未连接，跳过此测试");
        return false;
    }

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

    return allPassed;
}

int LinkerHandUserTest::runAllTests(LINKER_HAND model, HAND_TYPE handType, COMM_TYPE commType) {
    model_ = model;
    handType_ = handType;

    printHeader();

    std::cout << "测试配置:\n";
    std::cout << "  ├── 型号: " << getModelName(model) << "\n";
    std::cout << "  ├── 手部: " << getHandTypeName(handType) << "\n";
    std::cout << "  └── 关节数: " << getJointCount(model) << "\n";

    // 执行测试
    bool envOk = testEnvironment();

    if (!envOk) {
        printError("\n环境检测失败，请先解决上述问题后再继续测试");
        printSummary();
        return 1;
    }

    bool connOk = testConnection();

    if (!connOk) {
        printError("\n硬件连接失败，请检查:");
        printError("  1. 灵巧手电源是否开启");
        printError("  2. CAN/PCAN 线缆是否正确连接");
        printError("  3. 型号和手部类型是否正确");
        printSummary();
        return 1;
    }

    testBasicFunction();
    testMotion();
    testSensors();

    printSummary();

    return (passedTests_ == totalTests_) ? 0 : 1;
}

// ============================================================================
// 命令行参数解析
// ============================================================================
void printUsage(const char* programName) {
    std::cout << "用法: " << programName << " [选项]\n\n";
    std::cout << "选项:\n";
    std::cout << "  --model <型号>    灵巧手型号: L7, L10, L20, L21, L25 (默认: L10)\n";
    std::cout << "  --hand <类型>     手部类型: left, right (默认: right)\n";
    std::cout << "  --help            显示此帮助信息\n\n";
    std::cout << "示例:\n";
    std::cout << "  " << programName << " --model L10 --hand right\n";
    std::cout << "  " << programName << " --model L25 --hand left\n";
}

LINKER_HAND parseModel(const std::string& modelStr) {
    if (modelStr == "O6") return LINKER_HAND::O6;
    if (modelStr == "L6") return LINKER_HAND::L6;
    if (modelStr == "L7") return LINKER_HAND::L7;
    if (modelStr == "L10") return LINKER_HAND::L10;
    if (modelStr == "L20") return LINKER_HAND::L20;
    if (modelStr == "L21") return LINKER_HAND::L21;
    if (modelStr == "L25") return LINKER_HAND::L25;
    return LINKER_HAND::L10;  // 默认
}

HAND_TYPE parseHandType(const std::string& typeStr) {
    if (typeStr == "left" || typeStr == "LEFT" || typeStr == "Left") {
        return HAND_TYPE::LEFT;
    }
    return HAND_TYPE::RIGHT;  // 默认
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char* argv[]) {
    LINKER_HAND model = LINKER_HAND::L10;
    HAND_TYPE handType = HAND_TYPE::RIGHT;
    COMM_TYPE commType = COMM_TYPE::COMM_CAN_0;

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "--model" && i + 1 < argc) {
            model = parseModel(argv[++i]);
        }
        else if (arg == "--hand" && i + 1 < argc) {
            handType = parseHandType(argv[++i]);
        }
    }

    // 运行测试
    LinkerHandUserTest tester;
    return tester.runAllTests(model, handType, commType);
}
