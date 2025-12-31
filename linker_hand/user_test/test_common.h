/**
 * @file test_common.h
 * @brief LinkerHand SDK 测试通用工具
 *
 * 提供测试所需的通用函数和定义
 *
 * @copyright Copyright (c) 2025 灵心巧手（北京）科技有限公司
 */

#ifndef TEST_COMMON_H
#define TEST_COMMON_H

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
// 测试基类
// ============================================================================
class TestBase {
protected:
    LinkerHandApi* hand_;
    LINKER_HAND model_;
    HAND_TYPE handType_;
    int totalTests_;
    int passedTests_;
    std::vector<TestResult> results_;

public:
    TestBase() : hand_(nullptr), totalTests_(0), passedTests_(0) {}

    virtual ~TestBase() {
        if (hand_) {
            delete hand_;
            hand_ = nullptr;
        }
    }

    // ========================================================================
    // 辅助打印函数
    // ========================================================================
    void printHeader(const std::string& testName) {
#ifdef PLATFORM_LINUX
        std::cout << Color::CYAN << Color::BOLD;
#else
        Color::setColor(Color::CYAN);
#endif

        std::cout << "\n";
        std::cout << "═══════════════════════════════════════════════════════════════\n";
        std::cout << "              LinkerHand SDK - " << testName << "\n";
        std::cout << "═══════════════════════════════════════════════════════════════\n";

#ifdef PLATFORM_LINUX
        std::cout << Color::RESET;
#else
        Color::setColor(Color::RESET);
#endif
        std::cout << "\n";
    }

    void printSectionHeader(const std::string& title) {
#ifdef PLATFORM_LINUX
        std::cout << Color::YELLOW << Color::BOLD;
#else
        Color::setColor(Color::YELLOW);
#endif

        std::cout << "\n[测试] " << title << "\n";

#ifdef PLATFORM_LINUX
        std::cout << Color::RESET;
#else
        Color::setColor(Color::RESET);
#endif
    }

    void printTestResult(const std::string& testName, bool passed, const std::string& detail = "") {
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

    void printSuccess(const std::string& msg) {
#ifdef PLATFORM_LINUX
        std::cout << Color::GREEN << "  ├── " << msg << Color::RESET << "\n";
#else
        Color::setColor(Color::GREEN);
        std::cout << "  |-- " << msg;
        Color::setColor(Color::RESET);
        std::cout << "\n";
#endif
    }

    void printError(const std::string& msg) {
#ifdef PLATFORM_LINUX
        std::cout << Color::RED << "  ├── " << msg << Color::RESET << "\n";
#else
        Color::setColor(Color::RED);
        std::cout << "  |-- " << msg;
        Color::setColor(Color::RESET);
        std::cout << "\n";
#endif
    }

    void printWarning(const std::string& msg) {
#ifdef PLATFORM_LINUX
        std::cout << Color::YELLOW << "  ├── " << msg << Color::RESET << "\n";
#else
        Color::setColor(Color::YELLOW);
        std::cout << "  |-- " << msg;
        Color::setColor(Color::RESET);
        std::cout << "\n";
#endif
    }

    void printInfo(const std::string& msg) {
        std::cout << "  ├── " << msg << "\n";
    }

    void printSummary() {
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

    // ========================================================================
    // 辅助工具函数
    // ========================================================================
    std::string getModelName(LINKER_HAND model) {
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

    std::string getHandTypeName(HAND_TYPE type) {
        return (type == HAND_TYPE::LEFT) ? "左手 (Left)" : "右手 (Right)";
    }

    int getJointCount(LINKER_HAND model) {
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

    std::string vectorToString(const std::vector<uint8_t>& vec) {
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
    bool checkCanInterface(const std::string& ifname) {
        std::string path = "/sys/class/net/" + ifname;
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }
#endif

#ifdef PLATFORM_WINDOWS
    bool checkPcanDriver() {
        HMODULE hModule = LoadLibraryA("PCANBasic.dll");
        if (hModule != NULL) {
            FreeLibrary(hModule);
            return true;
        }
        return false;
    }
#endif

    // ========================================================================
    // 设备连接辅助函数
    // ========================================================================
    bool connectDevice(LINKER_HAND model, HAND_TYPE handType) {
        model_ = model;
        handType_ = handType;

        std::cout << "测试配置:\n";
        std::cout << "  ├── 型号: " << getModelName(model) << "\n";
        std::cout << "  ├── 手部: " << getHandTypeName(handType) << "\n";
        std::cout << "  └── 关节数: " << getJointCount(model) << "\n\n";

        printInfo("正在连接灵巧手...");

        try {
            hand_ = new LinkerHandApi(model_, handType_);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            std::string version = hand_->getVersion();

            if (!version.empty() && version != "0.0.0.0") {
                printSuccess("设备连接成功");
                printSuccess("固件版本: " + version);
                return true;
            } else {
                printError("无法获取版本信息");
                printError("请检查硬件连接和通信线路");
                return false;
            }

        } catch (const std::exception& e) {
            printError(std::string("连接失败: ") + e.what());
            return false;
        }
    }

    int getResult() {
        return (passedTests_ == totalTests_) ? 0 : 1;
    }
};

// ============================================================================
// 命令行参数解析
// ============================================================================
inline LINKER_HAND parseModel(const std::string& modelStr) {
    if (modelStr == "O6") return LINKER_HAND::O6;
    if (modelStr == "L6") return LINKER_HAND::L6;
    if (modelStr == "L7") return LINKER_HAND::L7;
    if (modelStr == "L10") return LINKER_HAND::L10;
    if (modelStr == "L20") return LINKER_HAND::L20;
    if (modelStr == "L21") return LINKER_HAND::L21;
    if (modelStr == "L25") return LINKER_HAND::L25;
    return LINKER_HAND::L10;
}

inline HAND_TYPE parseHandType(const std::string& typeStr) {
    if (typeStr == "left" || typeStr == "LEFT" || typeStr == "Left") {
        return HAND_TYPE::LEFT;
    }
    return HAND_TYPE::RIGHT;
}

inline void printUsageCommon(const char* programName, const char* testDescription) {
    std::cout << "LinkerHand SDK - " << testDescription << "\n\n";
    std::cout << "用法: " << programName << " [选项]\n\n";
    std::cout << "选项:\n";
    std::cout << "  --model <型号>    灵巧手型号: L7, L10, L20, L21, L25 (默认: L10)\n";
    std::cout << "  --hand <类型>     手部类型: left, right (默认: right)\n";
    std::cout << "  --help            显示此帮助信息\n\n";
    std::cout << "示例:\n";
    std::cout << "  " << programName << " --model L10 --hand right\n";
}

inline bool parseArgs(int argc, char* argv[], LINKER_HAND& model, HAND_TYPE& handType, const char* testDescription) {
    model = LINKER_HAND::L10;
    handType = HAND_TYPE::RIGHT;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsageCommon(argv[0], testDescription);
            return false;
        }
        else if (arg == "--model" && i + 1 < argc) {
            model = parseModel(argv[++i]);
        }
        else if (arg == "--hand" && i + 1 < argc) {
            handType = parseHandType(argv[++i]);
        }
    }
    return true;
}

#endif // TEST_COMMON_H

