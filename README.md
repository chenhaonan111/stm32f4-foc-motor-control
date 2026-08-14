# STM32F407 FOC Brushless Motor Control / STM32F407 无刷电机磁场定向控制

A field-oriented control (FOC) firmware for PMSM/BLDC motors on STM32F407, supporting both **sensored (encoder)** and **sensorless** (strong-drag startup + SMO back-EMF observer + PLL) operation with current / speed / position cascaded loops.

基于 STM32F407 的 PMSM/BLDC 磁场定向控制(FOC)固件,支持**有感(编码器)**与**无感(强拖启动 + SMO 反电势观测器 + 锁相环)**两种方式,包含电流 / 速度 / 位置三环级联控制。

---

## 功能特性 / Features

- **FOC 全链路**:Clark / Park / 反 Park 变换、SVPWM(含过调制处理)、双轴电流 PID(带 LPF、抗积分饱和)。
- **有感控制**:基于 TIM3 增量式编码器,实现电流环、速度环、位置环(三环级联)。
- **无感控制**:
  - **SMO 滑模观测器**:αβ 轴电流预测 + 饱和函数切换 + 反电势低通滤波,SPLL 锁相环提取电角度与转速,适用于中高速。
  - **强拖启动**:开环/闭环强拖将电机拉至观测器可工作转速,角度与速度收敛后平滑切入闭环,含失步检测与自动回退。
- **保护机制**:母线过压 / 欠压、相电流过流检测,故障态软件关断驱动使能(SD)。
- **参数在线辨识**:上电自动辨识定子电阻 Rs、电感 Ls,转子对齐(90°→0°)+ 编码器方向检测 + 零点标定。
- **调试**:USART1 + DMA 经 VOFA+(JUST_FLOAT 协议)实时上传观测波形。

## 芯片与开发环境 / MCU & Toolchain

| 项目 | 说明 |
|------|------|
| MCU | STM32F407(Cortex-M4,168 MHz) |
| IDE / 编译器 | Keil MDK-ARM,AC5(armcc) |
| 外设配置 | HAL 库(CubeMX 风格初始化,Core 目录) |

## 运行模式 / Run Modes

通过 `MC.Motor.RunMode` 选择运行模式(默认:`0x07` 强拖切 SMO 速度电流闭环)。

| 模式 | 代码 | 类型 | 说明 |
|------|------|------|------|
| 编码器校准 Encoder Calib | `0x00` | 有感 | 转子对齐与编码器零点标定 |
| 电流开环 Current Open Loop | `0x01` | 有感 | 开环电压驱动 |
| 电流闭环 Current Close Loop | `0x02` | 有感 | d/q 双轴电流 PID |
| 速度电流闭环 Speed Current Loop | `0x03` | 有感 | 速度外环 → 电流内环 |
| 位置速度电流闭环 Position Loop | `0x04` | 有感 | 位置 → 速度 → 电流 三环级联 |
| 强拖开环 Strong Drag Open | `0x05` | 无感 | 开环电压强拖启动 |
| 强拖闭环 Strong Drag Close | `0x06` | 无感 | 强拖 + 电流闭环 |
| 强拖切 SMO 速度电流 | `0x07` | 无感 | 强拖启动 → SMO 观测 → 速度电流闭环 |

> 速度量纲说明:强拖切换逻辑内部统一使用电气 rpm(机械 rpm × 极对数),切换阈值(2400 / 1700 / 1200)均为电气 rpm,在 `motor_publicdata.c` 的 `StrongDragToObs` 初始化中配置。

## 硬件与外设 / Hardware & Peripherals

| 外设 | 用途 |
|------|------|
| TIM1 | 中心对齐 PWM(20 kHz),三相互补输出 + 死区,PA8/9/10 + PB13/14/15 |
| ADC1(注入) | TIM1 TRGO 硬件触发,与 PWM 同步采样两相电流:Iu(PB0)、Iv(PA6) |
| ADC1(规则 + DMA) | 采样母线电压(PB1)、板载温度(PA0) |
| TIM3 | 编码器接口模式(正交解码,PC6/PC7) |
| TIM2 | 系统节拍(100 µs),调度按键 / LED / 串口任务 |
| USART1 + DMA | PB6/PB7,2 Mbps,VOFA+ JUST_FLOAT 调试波形上传 |
| GPIO | PE2/3/4 按键(短按/长按),PE0/1 LED,PF10 驱动使能(SD) |

- FOC 控制周期:`TS = 50 µs`(20 kHz)。
- 速度给定:按键步进调节(`SPEED_SET_DIR` 控制方向)。

## 电机参数 / Motor Parameters

| 参数 | 值 / 说明 |
|------|----------|
| 极对数 | 4 |
| 编码器线数 | 1000(×4 倍频) |
| 定子电阻 Rs | 上电自动辨识 |
| 定子电感 Ls | 上电自动辨识(表贴式 Ld≈Lq) |
| 编码器零点偏移 CalibOffset | 上电自动标定 |
| 最高机械转速 | 4000 rpm |
| 过流阈值 | 12 A |
| 母线电压范围 | 13 ~ 59 V |

> 电流采样:20 mΩ 采样电阻 + 6 倍放大;母线分压 1K / 24K。相关宏定义集中在 `User/MotorControl/Inc/motor_publicdata.h` 顶部,换电机时按需修改。

## 无感控制方案 / Sensorless Scheme

- **启动**:开环强拖(I-F),强拖电流按转速区间自适应调节。
- **切换**:SMO 观测 αβ 反电动势 → 归一化 PLL 提取电角度/电角速度 → 角度误差与速度同时收敛后切入闭环,切换瞬间速度环积分器用强拖电流初始化,电流无突变。
- **保护**:闭环失步(低速大角度误差)自动回退开环并限时恢复。

## 目录结构 / Directory

```
FOCProjectF407/
├── Core/               # 初始化代码:main / 中断 / 时钟 / 外设
├── Drivers/            # STM32F4 HAL 驱动 + CMSIS
├── MDK-ARM/            # Keil 工程(FOCProjectF407.uvprojx)
└── User/
    ├── Global/         # 全局初始化、中断回调调度、目标设定
    ├── MotorControl/   # FOC 核心:变换/SVPWM/PID/观测器/辨识/有感无感控制
    ├── KeyControl/     # 按键扫描
    ├── LedControl/     # LED 指示
    └── UsartControl/   # VOFA+ 串口调试输出
```

## 编译与烧录 / Build & Flash

1. 用 Keil MDK 打开 `MDK-ARM/FOCProjectF407.uvprojx`。
2. 选择 Target `FOCProjectF407`,点击 Build / Rebuild。
3. 通过 J-Link / ST-Link 下载生成的 `MDK-ARM/FOCProjectF407/FOCProjectF407.hex`。

> 上电流程:ADC 零点校准 → 参数辨识(Rs/Ls)→ 转子对齐与编码器标定 → 自动进入运行模式。

## 调试 / Debug

上位机使用 [VOFA+](https://www.vofa.plus),串口接收 JUST_FLOAT 协议数据(2 Mbps),可实时观察:

| 通道 | 内容 |
|------|------|
| ch1 | 编码器电角度(真实位置) |
| ch2 | SMO-PLL 观测电角度 |
| ch3 | PLL 鉴相误差 |
| ch4 | 开闭环状态(0=强拖开环,1=观测器闭环) |

## 状态 / Status

有感各模式与无感(强拖 + SMO)模式均已调通。

> 备注:高频注入(HFI)零速方案曾在本电机上评估——实测该电机凸极性仅在深饱和大电流注入下出现,解调无可用负序位置信号,经典方波注入不适用,相关代码已移除。如后续有零速无感需求,建议编码器方案或旋转注入类方法。
