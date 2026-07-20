# BalancedCar

基于 STM32F103C8T6 的两轮自平衡车。项目使用 STM32CubeMX 生成 HAL
外设初始化代码，以 CMake、GNU Arm Embedded Toolchain 和 ST-Link 进行
构建与调试。目前处于硬件驱动完成、控制算法尚未接入的 bring-up 阶段。

## 硬件组成

- STM32F103C8T6 最小系统板
- TB6612FNG 双路直流电机驱动
- 两路带 AB 相霍尔编码器的 MG310 减速电机
- MPU6050 六轴惯性传感器
- HC-04 蓝牙串口模块
- SSD1306 128×64 OLED
- 四个低电平有效按键和一个低电平有效 LED

底板原理图来源于嘉立创开源硬件项目“江协同款平衡小车底板”，本仓库的
引脚与软件配置以实际使用的该版本底板为准。

## 引脚映射

| 功能 | STM32 引脚 | 外设 |
| --- | --- | --- |
| 板载 LED | PC13 | GPIO，低电平有效 |
| 电机 A PWM | PA0 | TIM2_CH1 |
| 电机 B PWM | PA1 | TIM2_CH2 |
| 蓝牙 TX/RX | PA2 / PA3 | USART2 |
| 按键 K4/K3 | PA4 / PA5 | GPIO 输入，上拉 |
| 编码器 1 A/B | PA6 / PA7 | TIM3_CH1/CH2 |
| 按键 K2/K1 | PB0 / PB1 | GPIO 输入，上拉 |
| 编码器 2 A/B | PB6 / PB7 | TIM4_CH1/CH2 |
| OLED SCL/SDA | PB8 / PB9 | I2C1 Remap |
| MPU6050 SCL/SDA | PB10 / PB11 | I2C2 |
| TB6612 AIN1/AIN2 | PB12 / PB13 | GPIO 输出 |
| TB6612 BIN1/BIN2 | PB14 / PB15 | GPIO 输出 |

## 关键外设参数

- 系统时钟：72 MHz，APB1 为 36 MHz，APB1 定时器时钟为 72 MHz。
- TIM1：1 kHz 更新频率，预留给固定周期任务。
- TIM2：20 kHz 双路 PWM，`PSC=0`、`ARR=3599`。
- TIM3/TIM4：Encoder Mode TI12，16 位自由运行计数，两个输入滤波均为 15。
- USART2：9600 baud；RX DMA 为 Circular，TX DMA 为 Normal。
- I2C1：SSD1306，7 位地址 `0x3C`。
- I2C2：MPU6050，默认 7 位地址 `0x68`。

## BSP 结构

`BSP/` 只封装硬件访问和最小运行状态，不放入姿态解算、速度换算、PID
或蓝牙调参协议。

| 模块 | 设计边界 |
| --- | --- |
| LED | 封装 PC13 的低电平有效语义 |
| Key | 主循环轮询的非阻塞消抖，同时提供持续状态与单次边沿事件 |
| BlueToothSerial | USART2 空闲线循环 DMA 接收、环形缓冲和非阻塞 DMA 发送 |
| OLED | 移植并适配 SSD1306 的帧缓冲绘图驱动 |
| MPU6050 | 器件探测、寄存器配置和 14 字节原始数据突发读取 |
| Motor | 将 `-1000..1000` 逻辑命令映射为 TB6612 方向和 PWM |
| Encoder | 启动 TIM 编码器模式并返回处理过 16 位回绕的采样增量 |

左右电机和编码器都支持 `inverted` 配置。应用层应通过架空轮子测试，把
“小车前进”统一为正方向，而不是在 PID 中补偿接线与安装方向。

## 构建

需要 Ninja、CMake，以及能够在 `PATH` 中找到的 `arm-none-eabi-gcc`。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

产物位于 `build/Debug/BalanceCar.elf`。也可以直接由 CLion 加载根目录的
`CMakeLists.txt` 和 `CMakePresets.json`。

## 当前验证状态

已经在实物上确认：

- LED 与四个按键
- USART2 蓝牙收发和 DMA BSP
- SSD1306 OLED
- MPU6050 探测、配置和连续原始数据读取

Motor 和 Encoder BSP 已实现并通过编译，但仍需进行架空轮子的联合验收：
核对通道对应关系、正负方向、停止行为和左右计数差异。在此测试完成前，
不要直接进入闭环直立测试。

当前 `Tests/BluetoothDiagnostic.c` 是 bring-up 诊断入口，会在 OLED 上显示
MPU6050 数据，并提供蓝牙、按键和 LED 的检查功能。正式控制循环接入后，
该入口应由应用层状态机替换。

## 下一阶段

1. 电机与编码器联合验收及每圈计数标定。
2. 固定周期调度与运行安全状态机。
3. MPU6050 静止零偏校准和姿态角解算。
4. 直立环、速度环和转向环。
5. 蓝牙调参协议及运行数据记录。

`Legacy/` 保存教程原始代码，仅用于行为对照；新的实现位于 `BSP/`。
