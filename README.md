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

## 应用层结构

正式应用入口位于 `App/`。初始化、状态机和传感器采样集中在
`App.c` 的唯一整车状态机中；按键与蓝牙统一生成Command，独立的Command文件
只做无状态命令映射，不建立Handler对象。单轴姿态滤波、纵向串级控制、
安全检查和蓝牙协议保留小而明确的边界。OLED作为低频调参诊断页，
直接由App分8页渐进刷新，不再建立独立显示对象。完整数据流和实现顺序见
[应用层架构文档](Docs/ApplicationArchitecture.md)。

项目没有公开的整车对象或句柄；`main.c` 只调用 `App_Init()` 和
`App_Update()`，`App.c` 内部直接管理唯一一份运行数据。

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
- 两路电机正反转、停止和PWM输出
- 两路AB相编码器及电机/编码器联合方向测试

`Tests/BluetoothDiagnostic.c` 保留已经使用过的 bring-up 诊断代码，但不再
参与正式固件编译；`main.c` 当前进入 `App` 整车安全状态机。

蓝牙已兼容教程手机上位机的方括号滑杆协议，并以非阻塞DMA回传六路
`[plot,...]` 数据；具体滑杆名称和曲线顺序见
[蓝牙调参协议](Docs/BluetoothProtocol.md)。教程摇杆帧也已接入：LV控制目标
前进速度，RH控制目标偏航角速度，通信超时后两项目标自动归零。

## 下一阶段

1. 完成整车传感器采样和每圈计数标定。
2. 固定周期调度、静止零偏校准和姿态角解算。
3. 直立环、速度环和转向环。
4. 实车复核摇杆方向、缩放及运行数据记录。

`Legacy/` 保存教程原始代码，仅用于行为对照；新的实现位于 `BSP/`。
