# C Board Basic

基于 RoboMaster 开发板 C 型（STM32F407IGH6）的 FreeRTOS 工程，使用 STM32 HAL 和 `sp_middleware` 实现板载外设、DBUS 遥控器、IMU 姿态解算和双 GM6020 电机联动控制。

## 主要任务

所有应用任务在 `Src/freertos.c` 中创建，源码位于 `applications/`。

| 任务 | 实现内容 | 时序 / 接口 |
| --- | --- | --- |
| `buzzer_task` | 启动后依次发出 3000、4000、5000 Hz 三声提示音，每声 300 ms，声间间隔 100 ms；之后不重复播放 | TIM4 CH3，50% 占空比 |
| `led_task` | RGB LED 按红、绿、蓝循环切换 | TIM5，每色 300 ms |
| `uart_task` | 接收 DBUS 遥控器数据，更新摇杆、拨杆等状态；接收完成和串口错误后重新请求接收 | USART3，DMA / 空闲接收 |
| `imu_task` | BMI088 采样、陀螺仪零偏估计、Mahony 姿态解算，发布 yaw、yaw 角速度和有效时间戳 | SPI1，稳定运行目标周期 1 ms |
| `can_task` | 双 GM6020 电机反馈接收，位置环—速度环串级 PID，比例联动、手动拖动接管、R 标方向复位和输入掉线失能 | CAN1，目标周期 1 ms |
| `plotter_task` | 输出 A/B 电机目标与实际角度、累计 yaw 和 yaw 角速度，共 6 路波形数据 | USART1，约 100 Hz |
| `defaultTask` | 初始化 USB Device，随后循环延时 | USB CDC 已初始化；上述波形从 USART1 输出 |

任务周期为代码设定值，实际调度和通信频率受运行负载影响。中间件中存在的其他电机、裁判系统等模块，不代表当前应用已经启用这些功能。

## 开发环境与目录

- STM32CubeMX：外设配置文件为 `cboard.ioc`；系统时钟 168 MHz，外部晶振配置 12 MHz。
- CMake ≥ 3.22、Ninja、Arm GNU Toolchain（`arm-none-eabi-gcc`、`arm-none-eabi-g++`）；相关程序需加入 PATH。
- VS Code 可配合 CMake Tools 和 Cortex-Debug；烧录调试使用 OpenOCD。
- `Src/`、`Inc/`：初始化、HAL 回调和 FreeRTOS 任务创建。
- `applications/`：应用任务与控制逻辑。
- `sp_middleware/`：本工程所用的外设、电机和算法源码，应保留当前匹配版本。
- `Drivers/`、`Middlewares/`：STM32 HAL、CMSIS、FreeRTOS 和 USB 库。
- `cmake/`、`CMakeLists.txt`、`CMakePresets.json`：构建配置；`STM32F407xx_FLASH.ld` 与 `startup_stm32f407xx.s`：链接脚本和启动文件。
- `.vscode/launch.json`、`openocd.cfg`：调试配置；`build/`：构建产物，不作为源码提交。

## 硬件与通信配置

1. 使用 C 型开发板、DBUS 接收机和两台 GM6020，A 电机 ID=1、B 电机 ID=2，均接 CAN1。电机供电、拨码及电流控制模式应与所用硬件匹配。
2. 当前驱动选用 `RM_Motors::GM6020` 电流 / 力矩控制：反馈 ID 为 `0x205`、`0x206`，共用控制帧 `0x1FE`。不要按 `GM6020_V` 电压控制帧配置设备。
3. CAN1 为 1 Mbit/s（APB1 42 MHz，分频 3，位时间 1+10+3 TQ）；芯片引脚 PD0=RX、PD1=TX。按板卡接口标识接 CAN_H、CAN_L，检查总线终端和参考地；不要把电机总线直接接 MCU 逻辑引脚。
4. DBUS 使用 USART3，100000 baud、8 位有效数据、偶校验、1 位停止位（HAL 配置为 9B+偶校验）；PC11=RX、PC10=TX。按 C 板接收机接口和硬件说明接线。
5. 波形输出使用 USART1，921600 baud、8N1、无流控，PA9=TX、PB7=RX。USB 转串口使用匹配的 3.3 V 逻辑电平，接板 TX 到转换器 RX，并共地。
6. BMI088 使用 SPI1，片选 PA4、PB0；KEY 为 PA0，按下为低电平。

## 编译与烧录

在含 `CMakeLists.txt` 的工程根目录打开终端：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出为 `build/Debug/C_Board_Basic.elf`。Release 构建可将两条命令中的 `Debug` 改为 `Release`。

ZIP 中的 `build/` 是原环境生成的缓存和产物，迁移电脑后应重新生成。若提示缓存路径不一致，可先将旧 `build/` 移到工程外备份，再重新配置，不要据旧 ELF 判断新源码是否编译成功。

`openocd.cfg` 默认使用 CMSIS-DAP 和 `target/stm32f4x.cfg`。使用 ST-Link 时，将接口配置切换为文件中已有的 `interface/stlink.cfg`，只保留一种接口配置。连接调试器的 SWD、GND 和所需参考电压后，可使用 VS Code 的 `OpenOCD: Debug`（F5），或在工程根目录执行：

```powershell
openocd -f openocd.cfg -c "program build/Debug/C_Board_Basic.elf verify reset exit"
```

## 上电与操作步骤

1. 固定电机底座，预留转动空间，遥控器右拨杆置下档。检查供电和接线后上电；LED 循环变色，蜂鸣器播放三声提示音。
2. 保持 C 板静止：IMU 预热约 10 s，零偏采样约 4 s，再解算稳定约 2 s，正常情况下约 16 s 后可用。`dbg_imu_stage=2`、`linkage_imu_ready=1` 表示进入就绪阶段；提示音不代表 IMU 已就绪。
3. 确认 `dbg_a_alive`、`dbg_b_alive`、`dbg_remote_alive` 均为 1，IMU 数据持续更新。将两台电机的 R 标手动对齐到 C 板 R 标方向。
4. 右拨杆保持下档，先松开 KEY，再长按 KEY 至少 2 s。`dbg_calibrated=1` 表示记录成功。也可在调试器中将 `dbg_calibrate` 设置为 1，仍须满足输入正常、右下档的条件。
5. 选择下表中的工作模式。首次操作先使用较小转角，观察目标与实际角度、跟随方向和通信状态。
6. 停止控制时将右拨杆回下档；完全停止使用时按硬件操作规程断开电机电源。

标定只保存在 RAM，复位或断电后必须重新进行。掉线后，即使输入恢复，也必须回到右下档才能再次使能；未复位情况下源码保留此前标定。

### 遥控器拨杆

| 拨杆 | 档位 | 行为 |
| --- | --- | --- |
| 右拨杆 | 下 | 失能，命令力矩为 0；正常输入下允许标定和重新使能准备 |
| 右拨杆 | 中 | 联动：A 目标跟随 C 板累计 yaw 变化，B 目标按左拨杆比例变化 |
| 右拨杆 | 上 | R 标方向复位：依据标定和当前 C 板 yaw，沿最近角度路径对齐方向 |
| 左拨杆 | 下 | 联动比例 B/A = 0.5 |
| 左拨杆 | 中 | 联动比例 B/A = -1，反向等比例 |
| 左拨杆 | 上 | 联动比例 B/A = 3 |

进入模式或在联动中改变比例时，源码以电机实际位置建立新参考并清除 PID。复位计算假设两台电机底座固定、C 板独立转动；yaw 是相对姿态量，可能随时间漂移，不能作为绝对航向使用。

### 手动拖动与保护逻辑

- 手动拖动识别默认开启（`dbg_manual_enable=1`），只用于联动模式。C 板静止一段时间后，代码结合电机速度、位置误差及上一周期命令力矩与速度的乘积判断拖动；判定持续至少 80 ms 后，被拖动电机撤去命令力矩，另一台按比例跟随。停止拖动约 300 ms 后恢复双电机位置控制。
- 识别为经验阈值判断，不是外力传感器检测；安装或负载变化后需要重新核对参数。`dbg_manual_leader` 为 0/1/2，分别表示无接管/A/B。
- 任一电机或 DBUS 数据超过有效窗口（100 ms）、IMU 未就绪或数据过期（20 ms）、拨杆状态无效时，进入失能逻辑，清除 PID，并尝试发送零命令力矩。
- 软件零命令依赖 CAN 通信成功，不能替代硬件断电措施。运动过程中避免接触转子和机械夹点。

## 波形与调试

串口输出是二进制协议，并非文本打印：帧头 `AA BB`，随后 1 字节负载长度 `18`（十六进制，24 字节），再跟随 6 个小端 IEEE 754 float32，每帧 27 字节，无额外帧尾。上位机须匹配此协议，不能直接按文本或其他波形协议解释。

通道顺序：A 目标角度、A 实际角度、B 目标角度、B 实际角度、展开后的累计 yaw、yaw 角速度。角度单位 rad，角速度单位 rad/s。

常用调试变量：`dbg_mode`（0 失能/1 联动/2 复位）、`dbg_calibrated`、`dbg_ratio`、`dbg_a_rx_count`、`dbg_b_rx_count`、各 `dbg_*_alive`、`dbg_a_torque_cmd`、`dbg_b_torque_cmd`。默认目标速度限幅 15 rad/s、命令力矩限幅 0.3 N·m；PID 和 `A_SIGN`、`B_SIGN` 见 `can_task.cpp`。修改方向或控制参数后重新编译、烧录和标定。
