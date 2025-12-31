# LinkerHand-CPP-SDK

**[English](README.md)** | **[中文](README_CN.md)**

## 概述

LinkerHand-CPP-SDK 是由灵心巧手（北京）科技有限公司开发，用于 O6、L6、L7、L10、L20、L21、L25 型号灵巧手的驱动软件和 Demo 示例。

## 安装

### windows

系统：windows 11 64 位系统

环境：cmake 4.0.3、MinGW x86_64 15.1.0
- 下载 SDK

```bash
git clone https://github.com/linkerbotai/linker_hand_cpp_sdk.git
```

- 编译：

    - 1、拷贝 linker_hand_cpp_sdk/linker_hand/third_party/PCAN_Basic/x64/PCANBasic.dll 到 C:\Windows\System32 目录下

    - 2、拷贝 linker_hand_cpp_sdk/linker_hand/third_party/Robotic_Arm/windows/win_mingw64_c++_v1.1.0/libapi_cpp.dll 到 C:\Windows\System32 目录下

```bash
cd linker_hand_cpp_sdk/linker_hand
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
cmake --build .
```

- 运行示例

```bash
./linker_hand_example.exe
```

### ubuntu

- 下载 SDK

```bash
git clone https://github.com/linkerbotai/linker_hand_cpp_sdk.git
```

- 启动脚本
```bash
cd linker_hand_cpp_sdk/linker_hand
./script.sh
```
![alt text](linker_hand/img/script.png)
- 运行示例

```bash
cd build
./linker_hand_example
```

![alt text](linker_hand/img/example.png) 

## 快速开始

- 创建 main.cpp 文件，并添加以下代码：

```cpp
// main.cpp
#include "LinkerHandApi.h"

int main() {

    // 调用API接口
    LinkerHandApi hand(LINKER_HAND::L10, HAND_TYPE::RIGHT);

    // 获取版本信息
    std::cout << hand.getVersion() << std::endl;

    // 握拳
    std::vector<uint8_t> fist_pose = {101, 60, 0, 0, 0, 0, 255, 255, 255, 51};
    hand.fingerMove(fist_pose);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 张开
    std::vector<uint8_t> open_pose = {255, 104, 255, 255, 255, 255, 255, 255, 255, 71};
    hand.fingerMove(open_pose);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
```

- 创建 CMakeLists.txt 文件，并添加以下配置：

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(MyProject)

# 查找 LINKER_HAND_LIB 库
find_library(LINKER_HAND_LIB
    NAMES linker_hand_lib
    PATHS /usr/local/linker_hand_cpp_sdk/lib
    NO_DEFAULT_PATH
)

# 查找 RMAN_API_LIB 库
find_library(RMAN_API_LIB
    NAMES api_cpp
    PATHS /usr/local/linker_hand_cpp_sdk/third_party/Robotic_Arm/lib
    NO_DEFAULT_PATH
)

# 包含目录
include_directories(
	include
	/usr/local/linker_hand_cpp_sdk/include
	/usr/local/linker_hand_cpp_sdk/third_party/Robotic_Arm/include
)

# 添加可执行文件
add_executable(my_project main.cpp)

# 链接库
target_link_libraries(my_project ${LINKER_HAND_LIB} ${RMAN_API_LIB} pthread)
  ```

```
  🔧 核心SDK目录 (linker_hand/)

  linker_hand/
  ├── include/                   # 公共头文件 (18个)
  │   ├── LinkerHandApi.h       # ⭐ 主API门面 (入口点)
  │   ├── Common.h              # 枚举: LINKER_HAND, HAND_TYPE, COMM_TYPE
  │   ├── IHand.h               # 抽象手接口
  │   ├── HandFactory.h         # 工厂模式
  │   ├── LinkerHandL6.h        # L6型号
  │   ├── LinkerHandL7.h        # L7型号
  │   ├── LinkerHandL10.h       # L10型号
  │   ├── LinkerHandL20.h       # L20型号
  │   ├── LinkerHandL25.h       # L25型号
  │   ├── ICommunication.h      # 通信接口抽象
  │   ├── ICanBus.h             # CAN接口
  │   ├── CanBus.h              # Linux SocketCAN
  │   ├── CanBusFactory.h       # CAN总线工厂
  │   ├── CanFrame.h            # CAN帧定义
  │   ├── PCANBus.h             # Windows PCAN
  │   ├── EtherCAT.h            # EtherCAT通信
  │   ├── ModBus.h              # ModBus通信
  │   ├── ModbusLinkerHandL10.h # L10 ModBus专用
  │   └── RangeToArc.h          # 位置值<->弧度转换
  │
  ├── src/                       # 源代码实现 (12个)
  │   ├── LinkerHandApi.cpp
  │   ├── LinkerHandL6.cpp
  │   ├── LinkerHandL7.cpp
  │   ├── LinkerHandL10.cpp
  │   ├── LinkerHandL20.cpp
  │   ├── LinkerHandL25.cpp
  │   ├── CanBus.cpp
  │   ├── PCANBus.cpp
  │   ├── EtherCAT.cpp
  │   ├── ModBus.cpp
  │   ├── ModbusLinkerHandL10.cpp
  │   └── RangeToArc.cpp
  │
  ├── user_test/                 # ⭐ 用户硬件测试工具 (6个文件)
  │   ├── linker_hand_user_test.cpp  # 主入口
  │   ├── test_common.h              # 通用定义
  │   ├── test_environment.cpp       # 环境检测
  │   ├── test_connection.cpp        # 连接测试
  │   ├── test_basic_function.cpp    # 基础功能测试
  │   ├── test_motion.cpp            # 运动测试
  │   └── test_sensors.cpp           # 传感器测试
  │
  ├── examples/                  # 示例程序
  │   ├── Examples.cpp          # 综合示例
  │   ├── test.cpp              # 开发测试
  │   ├── test_l7.cpp           # L7测试
  │   ├── Demo_ModbusRTU.cpp    # ModBus RTU演示
  │   ├── L10/
  │   │   └── action_group_show.cpp
  │   ├── range_to_arc/
  │   │   ├── range_to_arc.cpp
  │   │   └── README.md
  │   └── ModbusRTU/            # 独立ModBus项目
  │       ├── CMakeLists.txt
  │       ├── main.cpp
  │       └── run.sh
  │
  ├── config/
  │   └── setting.yaml          # ⭐ 手型号/传感器配置
  │
  ├── third_party/
  │   ├── PCAN_Basic/           # Windows PCAN驱动
  │   │   ├── Include/          # 多语言头文件
  │   │   └── x64/              # DLL + LIB
  │   └── Robotic_Arm/          # 睿尔曼机械臂库
  │       ├── include/
  │       ├── linux/lib/
  │       └── windows/
  │
  ├── docs/
  │   └── API-Reference.md
  ├── img/
  ├── CMakeLists.txt
  └── script.sh
```
- 编译
```bash
cd example
mkdir build
cd build
cmake ..
make
```
- 运行

```bash
./my_project
```

- position 与手指关节对照表

```
L6/O6: ["大拇指弯曲", "大拇指横摆", "食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"]

L7:  ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]

L10: ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]

L20: ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]

L21: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","预留","预留","预留","预留","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

L25: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]
```

## 示例

| 序号 | 文件名称  | 描述                                           |
| :--- | :-------- | :--------------------------------------------- |
| 1    | Examples  | 示例集合（支持 L7、L10、L20、L21、L25 灵巧手） |
| 2    | ModbusRTU | 仅支持 L10 型号灵巧手（四代睿尔曼臂）          |

## API 文档

- [C++ API 文档](linker_hand/docs/API-Reference.md)

## 版本更新

