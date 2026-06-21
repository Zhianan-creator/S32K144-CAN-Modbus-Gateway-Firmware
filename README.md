# 基于 S32K144 的 CAN-Modbus 储能通信网关固件开发

本项目是一个基于 NXP S32K144 的储能通信网关固件项目，用于将 BMS 侧 CAN 数据解析后映射到 Modbus RTU 保持寄存器，并通过 RS485 提供给 PC、EMS、PLC 或调试工具读取。

## 硬件与软件平台

- 主控芯片：NXP S32K144
- 开发环境：S32 Design Studio
- CAN 通信：FlexCAN
- 串口与 RS485：LPUART + RS485 DE/RE 方向控制
- 上位机验证：Modbus Poll / 串口调试工具
- 主要协议：CAN、Modbus RTU

## 主要功能

- 接收并解析 BMS CAN 状态帧，当前核心状态帧 ID 为 `0x180`
- 将电压、电流、SOC、温度、状态位和故障位等数据写入 Modbus holding register
- 支持 Modbus RTU 从站功能，包含 `0x03` 读保持寄存器、`0x06` 单寄存器写入和 `0x10` 多寄存器写入
- 维护系统运行状态寄存器，包括运行时间、CAN 接收计数、Modbus 收发计数、CRC 错误、地址错误和 BMS 在线状态
- 支持 CAN 控制帧发送路径，当前控制帧 ID 为 `0x300`
- 提供 OLED 运行状态显示，用于现场调试 CAN、Modbus 和 BMS 数据状态

## 工程结构

```text
Sources/            应用层源码，包括 CAN、Modbus、RS485、OLED 和主循环逻辑
include/            应用层头文件
Generated_Code/     Processor Expert 生成的外设初始化代码
Project_Settings/   启动文件和链接脚本
SDK/                S32K144 SDK 驱动与平台支持代码
Documentation/      项目说明与代码阅读文档
```

## 构建方式

使用 S32 Design Studio 导入本工程，然后选择 `Debug_FLASH` 配置进行构建。

工程的核心源码位于 `Sources/` 和 `include/`，外设初始化和底层驱动依赖 `Generated_Code/`、`Project_Settings/` 和 `SDK/`。

## 调试与验证

- CAN 侧接收 BMS 状态帧，例如 `0x180`
- RS485 侧使用 Modbus Poll 读取 holding registers
- 常用验证方式是读取起始地址 `0`、数量 `8` 的 BMS 数据寄存器
- 系统状态寄存器可用于查看 CAN 接收、Modbus 收发、CRC 错误和 BMS 在线状态

## 开发说明

- `Sources/` 下是主要业务逻辑代码，可重点维护 CAN 解析、Modbus 映射、RS485 收发和 OLED 状态显示。
- `Generated_Code/` 和 `SDK/` 主要由工具链或 SDK 提供，修改前需要确认不会破坏 S32 Design Studio 工程配置。
- 本项目定位为 BMS CAN 数据到 Modbus RTU/RS485 的协议转换网关固件，不包含完整 BMS 采样与保护算法。
