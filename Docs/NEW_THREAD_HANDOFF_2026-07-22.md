# BalanceCar 新对话完整交接记录

> 生成时间：2026-07-22  
> 工程位置：`D:\HardWareProjects\BalanceCar\BalanceCar`  
> 用途：当前 Codex 对话频繁压缩上下文，因此把项目背景、硬件事实、代码现状、调试结论和下一步完整写在这里。新对话应先读本文件，再检查当前源码，不要从头猜测。

## 1. 用户的目标和协作方式

用户正在从零完成一辆 STM32 两轮自平衡车，主要目标不是复制教程，而是借项目学习：

- STM32 HAL、CubeMX、GCC、CMake、GDB/OpenOCD 一类现代嵌入式工具链；
- GPIO、定时器、PWM、编码器、I2C、UART DMA 等外设；
- MPU6050 采样、零偏校准、互补滤波；
- 角度环、速度环、转向环以及实际调参；
- 用合理而不过度的嵌入式软件架构替换教程里的 Keil/标准库面条代码。

用户希望大部分关键代码由自己写，通常更喜欢“橡皮鸭式”协作：先讲原理和下一小步，让用户自己实现，再检查。已经写过、纯样板或时间成本很高的部分，可以由 Codex 实现。

用户明确反感过度设计。后续代码应遵守：

- 不要为了形式套 `Config -> GetDefault -> Init -> initialized` 生命周期；
- 不要给一行公式套庞大对象；
- 不要把本来清楚的一段逻辑拆成大量微型函数；
- 不要重复安全层、事件层、状态进入层；
- 优先采用“物理能力/控制对象”抽象，而不是通用业务软件式抽象；
- 保留一个清楚的整车状态机和直接的数据流；
- 对当前规模，简单、可读、可调试比高度通用更重要。

用户学习背景：学过 CS61A/61B；独立做过 WS2812 点彩灯项目，包含 PWM、按键、UI、OLED、蓝牙；画过 USB 2.0 Hub PCB；跟着画过但没完成 STM32 开发板。用户不是纯新手，可直接解释底层原理，但久未接触的 STM32 外设概念需要复习。

## 2. 重要操作约束

- 不要使用 Codex 浏览器工具。此前只要调用浏览器，Codex 桌面端就可能频繁关闭。除非用户明确说明问题已修复。
- 文件修改必须谨慎保留用户已有改动。当前工作区很脏，不能 reset、checkout 或覆盖用户工作。
- 用户之前在对话中贴过火山引擎/ASR 一类访问凭据。本文件故意不保存任何密钥。相关凭据应视为已经泄露，建议撤销或轮换。
- 新对话不要仅根据旧教程或旧文档推断代码；先看当前源码和 `git diff`。已有说明文档可能落后于后续调参修改。
- 本项目不是教程原车板，所有引脚和硬件判断必须以实际江协同款底板原理图及当前 `BalanceCar.ioc` 为准。

## 3. Git 和工作区现状

仓库：`D:\HardWareProjects\BalanceCar\BalanceCar`

远端：`https://github.com/whycantiusemyname/BalancedCar`

最近已知 HEAD：

```text
8270190 (main, origin/main) docs: document hardware and BSP status
```

最近一次检查时存在大量未提交改动：

```text
 M BSP/Inc/oled.h
 M BSP/Src/mpu6050.c
 M BSP/Src/oled.c
 M BalanceCar.ioc
 M CMakeLists.txt
 M Core/Src/i2c.c
 M Core/Src/main.c
 M README.md
 M Tests/BluetoothDiagnostic.c
?? App/
?? Docs/
```

这些改动是当前可运行控制程序的一部分，不是垃圾文件。尤其 `App/` 和 `Docs/` 还可能完全未被 Git 跟踪。不要执行任何会丢失它们的命令。

之前已经按驱动阶段做过若干提交，但整车 App、控制器和最新调试代码尚未形成可靠提交。稳定后应先审查 diff，再分主题提交。

## 4. 工具链和 CLion

用户希望抛弃 Keil，使用 CubeMX + GCC + CMake + CLion/VS Code。

已确认 GCC 位于：

```text
D:\Program Files\STMicroelectronics\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe
```

常用构建命令：

```powershell
& 'D:\Program Files\STMicroelectronics\STM32CubeCLT_1.22.0\CMake\bin\cmake.exe' --build --preset Debug -j 22
```

常用烧录目标：

```powershell
& 'D:\Program Files\STMicroelectronics\STM32CubeCLT_1.22.0\CMake\bin\cmake.exe' --build build\Debug --target flash
```

CMake 使用 `cmake/gcc-arm-none-eabi.cmake`。CLion 曾错误调用自己的 MinGW GCC：

```text
D:\Program Files\CLion 2026.1\bin\mingw\bin\gcc.exe
```

于是编译 `syscalls.c` 时报告 `sys/times.h` 不存在。这不是 STM32 库缺失，而是 CLion Profile 没用 ARM 工具链。应选择项目 CMake Preset/ARM toolchain，不能用桌面 MinGW 编译 MCU 工程。

CLion 2026.1 还偶发 JetBrains 自身的 Kotlin coroutine/NPE 启动错误，关闭再打开通常正常。这不是 Codex 修改 CLion 导致的。

最近一次检查没有在 PATH 中找到独立 `openocd`。不要在新对话里直接声称 OpenOCD 已经装好。现有 `flash` 目标此前可以正常烧录，可能走 STM32CubeProgrammer/工程脚本。

## 5. 实际硬件

实际主板是嘉立创开源硬件平台上的“江协同款平衡小车底板”，不是原教程板：

```text
https://oshwhub.com/hanlingmao/jiang-xie-tong-kuan-ping-heng-xiao-che-di-ban
```

主要模块：

- STM32F103C8T6 最小系统板；
- TB6612FNG 双路电机驱动模块；
- MPU6050 模块；
- HC-04 BLE 串口蓝牙模块；
- SSD1306 128x64 OLED，实际 I2C 地址为 `0x3C`，不是 `0x3D`；
- 两个 MG310 直流减速电机；
- 两路 AB 相霍尔编码器；
- 四个按键；
- 一个用户 LED；
- 两节锂电池串联供电。

无线 NRF24L01 模块目前未使用。USB 转串口模块也没有购买，当前使用蓝牙即可调参和看曲线。

### 5.1 实际引脚映射

| 功能 | STM32 引脚/外设 | 备注 |
|---|---|---|
| LED | PC13 | 低电平点亮 |
| 左/右电机 PWM | PA0 TIM2_CH1、PA1 TIM2_CH2 | 当前代码应核对左右命名 |
| 蓝牙 TX/RX | PA2 USART2_TX、PA3 USART2_RX | 9600 baud，DMA |
| K4/K3 | PA4、PA5 | 按下接地，输入上拉 |
| K2/K1 | PB0、PB1 | 按下接地，输入上拉 |
| 编码器 1 | PA6/PA7 TIM3 | AB 相 Encoder Mode |
| 编码器 2 | PB6/PB7 TIM4 | AB 相 Encoder Mode |
| OLED | PB8/PB9 I2C1 remap | SSD1306，0x3C |
| MPU6050 | PB10/PB11 I2C2 | 400 kHz |
| TB6612 AIN1/AIN2 | PB12/PB13 | 方向控制 |
| TB6612 BIN1/BIN2 | PB14/PB15 | 方向控制 |

### 5.2 时钟和外设配置

- SYSCLK/HCLK：72 MHz；
- APB1：36 MHz，但 APB1 定时器时钟因为预分频不为 1，所以为 72 MHz；
- APB2：72 MHz；
- TIM2 PWM：20 kHz，PSC=0，ARR=3599；
- TIM3/TIM4：Encoder Mode TI12，数字滤波约为 15；
- USART2：9600；
- USART2 RX：DMA1 Channel 6，Circular；
- USART2 TX：DMA1 Channel 7，Normal；
- I2C1/I2C2：400 kHz。

OLED 没有使用 DMA。STM32F1 固定 DMA 映射下，I2C1 DMA 与 USART2 DMA 通道存在冲突；而 OLED 刷新本身不在 5 ms 控制关键路径，所以采用分页面渐进刷新更合理。

## 6. 机械和装配结论

- 用户最后使用了 48 mm 轮子和 3 mm D 轴联轴/轮毂。
- 用户自己 DIY 了底板并安装电机支架。初期两边大约有 1 mm、2~3 度的偏差，后来调整到肉眼基本看不出歪斜。
- 这种小机械误差不需要独立“机械偏差环”。角度平衡、速度环、平衡角 trim 和转向环会处理大部分影响。
- 原 PCB 上直接安装电机支架的位置与通孔元件引脚干涉，所以最终使用额外 DIY/亚克力类底板是合理方案。
- PCB 背面铜字与若干接地缝合过孔碰到。那些过孔都是 GND，铜字也没有跨入其他网络，主要是外观问题，不影响功能。
- 编码器 6 芯排线最初因为插座/端子方向反了，导致供电与信号脚错误。后来已经换线/重新排针，编码器全部正常。
- P2 是右电机，P1 是左电机。

## 7. 已经完成并验证的硬件/BSP

以下项目都至少进行过单项测试：

- LED：正常；
- 四个按键：正常，非阻塞扫描/事件方式；
- 蓝牙：USART2 DMA 收发正常；
- OLED：SSD1306、地址 0x3C，正常显示；
- MPU6050：WHO_AM_I、寄存器读写、原始加速度/陀螺仪读取正常；
- 电机：左右、正反转、PWM 输出正常；
- 编码器：两路 AB 相计数正常；
- 逻辑前进方向：两个电机和两个编码器已经统一；
- 两轮同向测试时编码器都为正，计数只相差个位数。

MG310 编码器参数：电机端约 13 线，减速比约 20.4，因此输出轴一圈约 `13 * 20.4 = 265.2` 个基础计数；实际使用 TIM Encoder Mode 后倍率和代码单位要以当前驱动实现为准，不能只按商品页数字猜。

## 8. MPU6050 重要结论

用户的 MPU6050 上电时原始陀螺仪偏移曾出现：

```text
GX 312
GY -2505
GZ -364
```

其中 GY 固定偏移较大，但静止重复采样的变化很小；按压模块/PCB 时 GY 变化约在 ±200 raw 以内。它可能是兼容芯片或个体零偏较大，但只要稳定、量程和噪声正常，启动校准后仍可用于控制。

后来静止校准后：

- 残余角速度基本在很小范围；
- GZ 的静态偏置实测约为 `-2.85 dps`；
- 校正后的约定为左转正、右转负；
- 前倾 Pitch 为正；
- 前倾对应的 Gyro 也为正；
- 正的电机命令表示向前。

MPU 配置：

- DLPF_CFG 约为 3；
- Sample Rate Divider 约为 4；
- 内部 1 kHz 经分频后约 200 Hz；
- 对应控制周期 5 ms。

互补滤波使用：

- 陀螺仪积分负责短时间动态；
- 加速度角负责长期绝对参考；
- 当前时间常数约 5 s，在 5 ms 周期下 alpha 接近 0.999；
- 教程里的 0.999 并不神秘，它只是对应较慢地相信加速度角。alpha 不能脱离采样周期比较。

加速度安装角补偿曾设置约 `+1.5°`。靠墙/竖直静态测试后，用户认为地面零点误差大约在 ±1° 内。后续又通过实际平衡把 BalanceTrim 从早期的 -8°、-4° 一路调到约 -0.5°。不要把早期极端 trim 当成最终硬件事实。

## 9. 当前软件架构

主程序现在应当非常薄：

```text
main.c
  -> App_Init()
  -> while (1) App_Update()
```

BSP 大致包含：

- LED
- Key
- BluetoothSerial
- OLED
- MPU6050
- Motor
- Encoder

App 层大致包含：

- `App.c`：唯一整车运行时和整车状态机；
- `Command.c`：把蓝牙/按键输入统一成简单命令并执行；
- `BluetoothProtocol.c`：解析方括号协议，发送绘图数据；
- `PID.c`：通用单个 PID；
- `LongitudinalControl.c`：速度外环串联角度内环。

之前曾有 AttitudeEstimator、SafetyMonitor、AppDisplay、MotorMixer、大量事件和 Config 生命周期等抽象，后来已经按用户要求简化或合并。

当前没有单独的 `AttitudeEstimator` 对象；互补滤波直接写在 App 的控制采样路径中。也没有独立 SafetyMonitor 对象，基础的倾倒判断和电机停机由主状态机负责。

### 9.1 状态机

整车状态至少包括：

- `CALIBRATING`：启动静止校准；
- `BALANCING`：闭环平衡；
- `FALLEN`：倾倒，持续执行电机停止；
- `FAULT`：故障，持续执行电机停止。

曾经有 READY 状态，但用户指出平衡车在 READY 关闭电机时本来就会倒，因此删除。校准完成后直接进入 BALANCING。

状态机原则：进入某状态后，该 case 自己持续执行该状态动作。例如 FALLEN case 里持续 `Motor_Stop()`，不能依赖一个额外 `App_EnterState()` 偷偷做一次清零。这一问题已经被纠正。

### 9.2 非阻塞调度

使用 `HAL_GetTick()` 的协作式 deadline 调度：

- IMU + 角度环：5 ms，200 Hz；
- 速度环 + 转向环：20 ms，50 Hz；
- 按键：10 ms；
- 蓝牙轮询：5 ms；
- 蓝牙绘图：50 ms；
- OLED：约每 20 ms 刷一页，完整一帧约 160 ms。

控制运行时间曾用 DWT 实测：

```text
控制周期 min/max: 5 ms
最坏一次控制执行: 40080 cycles
72 MHz 下约 0.557 ms
约占 5 ms 周期的 11.1%
```

DWT 临时测量代码后来已移除。

## 10. 控制结构和数据流

### 10.1 纵向控制

速度环和角度环是串联关系：

```text
目标速度
  -> 速度 PI
  -> 目标俯仰角增量
  -> 与 BalanceTrim 相加
  -> 角度 PD/PID
  -> 公共平衡电机输出
```

速度环输出的不是 PWM，而是目标 Pitch 修正量。角度环才输出公共电机命令。

### 10.2 转向控制

转向环与纵向角度输出并联，最后差速混合：

```text
left  = balance_output - turn_output
right = balance_output + turn_output
```

转向环当前使用 GZ/yaw rate，属于角速度闭环，不是航向角保持。它可抑制自发转向，并跟随摇杆目标转速，但没有磁力计时无法长期记住绝对朝向。

转向输出有限幅，已知设计中约为 ±300；同时会根据纵向输出的剩余余量限制：

```text
turn_headroom = 1000 - abs(balance_output)
```

当目标转向为 0 且 `|GZ| < 1 dps` 时，会清理转向 PID 历史，防止微小零偏长期积累后带动车辆慢走。

## 11. 方向和单位约定——不可再搞错

历史上最大的调试错误是编码器方向与电机逻辑方向不一致：

- 两轮机械前进时，原始编码器一正一负；
- 直接取平均后互相抵消；
- 速度环几乎看不到车辆在移动；
- 导致前期速度环调参全部无效。

后来已修正编码器反向配置。还发现诊断命令 `B150` 曾经方向反了，也已纠正。

当前必须维持的统一约定：

- 前倾 Pitch：正；
- 前倾 Gyro：正；
- 逻辑前进电机命令：正；
- 两个轮子向前滚动时，两路编码器速度：都为正；
- 左转 GZ：正；
- 右转 GZ：负。

如果再改电机接线、左右轮映射、TIM 编码器配置或 Motor inversion，必须重新做以下最小测试：

1. 单独让左轮逻辑前进，编码器必须为正；
2. 单独让右轮逻辑前进，编码器必须为正；
3. 两轮同时逻辑前进，两者都正且数量级接近；
4. 车身前倾时，电机应朝倾倒方向追车身；
5. 左扭车身时，校正后 GZ 为正。

## 12. 电机死区

双轮正反转起步阈值实测大约都是 48，因此当前使用：

```text
Offset / deadzone compensation = 48
```

补偿方法应为：控制输出非零时，按符号附加 48；为零时保持零。不能在 0 命令时也强行输出 48。

有一个轮子的某段转子位置更难启动，同一圈不同位置可能相差不到约 20。当前选择统一 48 是可接受折中。不要为了一个局部硬点把全局 deadzone 抬得很高，否则会增加低速抖动。

## 13. 当前有效控制参数

最后已知、能够自主站立的一组参数：

```text
AngleKp       = 65
AngleKi       = 0
AngleKd       = 1.9

SpeedKp       = 0.003
SpeedKi       = 0.00015
SpeedKd       = 0

TurnKp        = 0（开始调转向时会临时设为约 1）
TurnKi        = 0
TurnKd        = 0

Offset        = 48
BalanceTrim   = -0.5 deg
```

配置中还可能存在：

```text
max tilt                  35 deg
max target pitch          10 deg
speed integral pitch cap   1 deg
motor output limit       1000
turn output limit         300
yaw deadband                1 dps
command timeout           500 ms
calibration samples       500
left motor inverted       false
right motor inverted      true
left encoder inverted     true
right encoder inverted    false
```

这些值以当前 `App/Inc/BalanceCarConfig.h` 为最终准则；本记录是最后已知状态，可能在用户最后几分钟调参中略有变化。

## 14. 调参过程的真实结论

### 14.1 角度环

早期由于单位、死区和机械差异，Kp 必须到几十，而不是教程里的 1 左右。这不是异常：教程和当前程序的角度单位、PWM 标度、采样周期、PID 离散形式、电机与轮径都不相同，参数数值不能横向照抄。

角度环最终大致在：

```text
Kp 45~65
Kd 1.9~2.4
```

范围内能稳定。Kd 太低时撞箱/扰动后出现来回抽动；Kd 太高时会造成高频锯齿和噪声放大。用户最后选择约 1.9，Kp 65。

### 14.2 平衡角

早期把 BalanceTrim 调到 -8.6° 时，车能暂时一动不动，但这主要是在掩盖传感器安装角、速度环失效和方向问题。后续修正后 trim 已回到约 -0.5°。

平衡角不必等于几何 0°，它代表轮轴接地点正上方的真实重心平衡位置，受电池、上层结构和传感器安装角影响。但如果需要 7~9°，应先怀疑传感器零点、机械装配和环路方向，不要直接永久接受。

### 14.3 速度环

速度环修正前，由于两编码器方向相反，平均速度错误；那时所有速度 Kp/Ki 结论都作废。

编码器方向修正后：

- `SpeedKp = 0.003` 可明显让车辆在受推后回收速度；
- Kp 太大时会形成慢速来回摆动；
- `SpeedKi` 从 0 逐步调到 0.0001、0.00015、0.0002；
- 约 `0.00015` 后车辆基本能自主站立；
- 0.0002 与 0.00015 肉眼差异很小；
- 速度积分的作用是消除长期缓慢漂移，不应指望它改变快速角度动态。

最终用户报告：车辆已能自主立起来，只剩刷式电机、死区带来的小抖动，以及未启用转向环时偶发自发转向。

### 14.4 波形颜色在不同阶段的含义

蓝牙绘图曾多次更换曲线定义，不能仅凭颜色猜。最后调转向前后的协议需要看当前 `BluetoothProtocol.c`。

已知某个调转向阶段的三条曲线设计为：

- 红色：校正后的 GZ，可能做了 `/10` 缩放；
- 绿色：turn output，可能做了 `/100` 缩放；
- 蓝色：forward speed，可能做了 `/1000` 缩放。

更早的阶段红/绿主要是目标角与当前角；后来又加入蓝色速度。新对话在解释截图前，必须先确认当前发送顺序和缩放。

## 15. 蓝牙协议现状

蓝牙 DMA 已经是非阻塞方案。手机端不一定发换行，协议应以完整方括号帧识别，不依赖 `\n`。

已支持或曾支持：

```text
[slider,AngleKp,value]
[slider,AngleKi,value]
[slider,AngleKd,value]
[slider,SpeedKp,value]
[slider,SpeedKi,value]
[slider,SpeedKd,value]
[slider,TurnKp,value]
[slider,TurnKi,value]
[slider,TurnKd,value]
[slider,Offset,value]
[slider,BalanceTrim,value]
```

按钮：

```text
[key,Start,down]
[key,Stop,down]
[key,Recover,down]
```

诊断电机命令曾设计为：

```text
[motor,L,150]
[motor,R,150]
[motor,B,150]
```

只允许在 FALLEN/非平衡状态运行约 300 ms，避免调试时突然冲车。

原教程/当前手机上位机摇杆帧格式：

```text
[joystick,LH,LV,RH,RV]
```

四个轴通常为 -100 到 100：

- 左摇杆竖轴 LV：前进/后退；
- 右摇杆横轴 RH：左转/右转。

截至本交接文档，摇杆帧仍很可能只是被识别后丢弃，尚未真正映射到目标速度和目标 yaw rate。这是当前最明确的待办。

## 16. 当前转向环进度

转向环刚开始调：

- 已测 GZ 零偏约 -2.85 dps，并写入/通过校准修正；
- 校正后左扭正、右扭负；
- TurnKp 约 1 时，用手扭车身可以看到回正趋势；
- 手扭法不适合判断实际转弯手感，用户要求加入摇杆后再调；
- TurnKi、TurnKd 仍为 0；
- 未开启稳定转向环时，车偶尔会自己缓慢转向；
- 开启转向环后若车开始慢走，要检查差速输出是否被错误混入公共纵向输出、GZ 零偏、积分残留以及左右轮增益差，而不是盲目调速度环。

最新用户结论：必须加上摇杆，通过命令转弯来测试灵敏度，而不是继续用手左右扭。

## 17. 新对话第一项任务：实现摇杆控制

建议按当前简化架构完成，不要新建复杂对象。

### 17.1 建议的数据命令

增加一个简单原子命令，例如：

```c
BALANCE_CAR_COMMAND_SET_MOTION_TARGET
```

负载只需要：

```c
struct {
    float forward_speed_counts_per_s;
    float yaw_rate_dps;
} motion;
```

一个摇杆帧同时更新两个目标，避免先更新速度、后更新转向的短暂不一致。

### 17.2 建议映射

- 解析并校验 `LH, LV, RH, RV` 四轴；
- 输入范围裁剪到 -100..100；
- 中心死区先用 5%；
- `LV` 映射到前进目标速度，初始可用 ±1000 counts/s；
- `RH` 映射到目标 yaw rate，初始可用 ±120 dps；
- 手机右摇杆向右通常 RH 为正，而内部约定右转 GZ 为负，因此可先用：

```text
target_yaw_rate = -RH * 120 / 100
```

- `command timeout = 500 ms` 已存在；超时必须把速度目标和转向目标都归零。

### 17.3 建议转向调试波形

调转向时暂时画：

```text
实际校正 GZ / 10
目标 yaw rate / 10
turn output / 100
```

这样可以直接判断：目标是否到达、方向是否正确、Kp 是否太小、输出是否饱和。

测试顺序：

1. 先让 LV 和 RH 都居中，确认不会慢走或自转；
2. TurnKp 约 1，TurnKi=0，TurnKd=0；
3. RH 分别推 25%、50%、100%，看 GZ 是否同方向跟随目标；
4. 若方向相反，只改一次统一符号，不要同时反转 GZ、混控和摇杆映射；
5. 先调 Kp 到转弯响应足够，再考虑很小的 D；
6. 暂时不要加转向积分，陀螺仪角速度环一般不需要 I，I 很容易造成松手后持续旋转。

可能涉及的文件：

```text
App/Inc/BalanceCarConfig.h
App/Inc/BalanceCarTypes.h
App/Src/Command.c
App/Src/BluetoothProtocol.c
App/Inc/BluetoothProtocol.h
App/Src/App.c
Docs/BluetoothProtocol.md
Docs/ApplicationArchitecture.md
README.md
```

在修改这些文件前应先亲自读当前实现，因为本对话最后尚未落地摇杆功能。

## 18. OLED 现状

- SSD1306，I2C 地址 0x3C；
- 原厂家驱动经过适配，字体表被扩充，因此能显示比用户以前驱动更多字符；
- 一次完整 ShowFrame 可能不算很慢，但平衡控制不应被整帧阻塞；
- 当前采用按页渐进刷新；
- BALANCING 时可能限制或关闭 OLED 刷新以降低控制抖动，需看当前 App 实现；
- OLED 主要用于硬件诊断和脱机查看，正常闭环调参以蓝牙波形为主。

## 19. 之前出现过、不要重复的错误

1. 用原教程板引脚代替实际 OSHWHub 板引脚。
2. CLion 用 MinGW GCC 编 STM32，误以为缺少 `sys/times.h`。
3. 把两路物理前进的编码器一正一负直接平均，导致速度近似为 0。
4. 同时修改电机方向和编码器方向，结果无法判断到底哪一层反了。
5. 根据某张旧截图颜色猜曲线，而不检查当前 telemetry 顺序。
6. 用过大的 BalanceTrim 掩盖传感器安装角或反馈方向问题。
7. 在速度环方向/编码器单位未确认前反复调 Kp/Ki。
8. 将所有防御性逻辑拆成 SafetyMonitor、EnterState、Mixer 等小层，架空主状态机。
9. 给简单互补滤波写复杂对象和生命周期。
10. 把速度环输出直接当 PWM；当前设计里它应是目标 Pitch 增量。
11. 认为参数必须与教程同数量级。单位、采样周期、输出标度不同，PID 数值没有直接可比性。
12. 在转向角速度环中急着加 Ki。GZ 零偏和差速不对时，积分只会使问题更明显。

## 20. 教程和理论学习进度

PID 系列视频已经基本看完，用户已经理解：

- P：即时误差；
- I：累计历史、消除长期稳态偏差；
- D：变化趋势/阻尼；
- 纯 P 是否有稳态误差取决于被控对象和扰动，不是“位置一定没有、速度一定有”的绝对定律；
- 微分先行/对测量微分可减小设定值突变冲击；
- 不完全微分是给 D 加低通，避免噪声放大；
- 死区可抑制微小抖动，但会损失精度；
- 积分限幅和 anti-windup 很重要；
- 变速积分与 AI 激活函数只是数学形状上可能相似，不等于同一原理；
- 梯度下降是优化参数的方法，PID 本身不是梯度下降。

当前项目无需继续全量看倒立摆器材演示，价值最高的是对照当前车的传感器、编码器、串级控制、蓝牙调参和故障分析。

## 21. 更早的相关项目背景

以下不是当前立刻要做，但能解释之前决策：

- 最初已把平衡车资料解压，比较过原理图/BOM，最后决定主板以实际 OSHWHub 版本为准；
- 曾讨论自己画遥控器 PCB，结论是以用户经验完全可行，难度中等，但当前先用手机蓝牙完成车辆控制；
- 提供的资料中没有找到明确的蓝牙调参手机 APK，当前使用用户已有的蓝牙串口小程序/应用；
- 曾考虑用游戏手柄经电脑蓝牙转发到 HC-04，技术上可行，但链路更复杂、延迟和映射调试较多，不是当前最短路径；
- 用户关注传统控制与具身智能。结论是 VLA/RL/世界模型主要改变高层决策、技能学习和规划，电机、姿态、力矩等快速低层闭环仍大量依赖经典控制、状态估计与优化控制；
- 用户不想只做“拼模块”，当前项目真正值得学的是实时数据流、符号/单位、系统辨识、调参方法、故障定位、工具链和合理软件架构。

另有电烙铁 160x40 开机图、魔圆相关像素图、PCB 铜字等支线任务，均与当前 BalanceCar 控制代码无关，新对话可忽略。

## 22. 建议的新对话开场语

可直接对新 Codex 说：

```text
请先完整阅读 D:\HardWareProjects\BalanceCar\BalanceCar\Docs\NEW_THREAD_HANDOFF_2026-07-22.md，
然后检查当前 git status 和 App/BSP 的实际代码。不要使用浏览器，不要重构现有架构，也不要丢弃未提交改动。
当前车已经能自主平衡，下一步是把 [joystick,LH,LV,RH,RV] 接入现有 Command，
用 LV 控制目标速度、RH 控制目标 yaw rate，并通过蓝牙画目标 GZ、实际 GZ、turn output 来调转向环。
先告诉我你从代码确认到的现状和最小修改方案，再动手。
```

## 23. 当前完成度总结

项目已经跨过最困难的硬件通断和基础平衡阶段：

- 硬件模块均已验证；
- BSP 已基本完成；
- 主循环和状态机已完成；
- MPU 校准与互补滤波已完成；
- 电机方向、编码器方向已经统一；
- 角度环已稳定；
- 速度环已能让车辆自主站立；
- 蓝牙在线调参和绘图可用；
- 转向环框架已存在但尚未用摇杆完整调通。

下一阶段不应再大改架构。重点是：摇杆接入、转向环定向与增益、前进/后退速度目标、参数固化、清理诊断代码、补文档、提交 Git。
