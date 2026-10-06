# C Board Basic

## 项目说明

本项目基于 RoboMaster 开发板 C 型进行阶段任务开发，
使用 STM32F407IGH6、FreeRTOS 和 sp_middleware 完成基础外设与通信功能。

## 当前开发状态

当前主要完成代码编写与编译检查，尚未进行实板验证。

### 已完成代码

- [x] 蜂鸣器控制代码
- [x] LED 流水灯代码
- [x] 串口打印代码
- [x] DBUS 遥控器接收代码

### 待完成验证

- [ ] 蜂鸣器实板验证
- [ ] LED 实板验证
- [ ] 串口打印实板验证
- [ ] DBUS 遥控器实板验证

## 开发环境

- MCU：STM32F407IGH6
- 开发板：RoboMaster C Board
- RTOS：FreeRTOS
- 中间件：sp_middleware
- 配置工具：STM32CubeMX
- 开发工具：VS Code

## 注意事项

1. 当前功能尚未全部进行实板验证。
2. 烧录前应检查 CubeMX 外设配置是否与开发板一致。
3. 测试前应检查供电、串口、CAN 及其他外设接线。
4. sp_middleware 应保持与当前工程匹配的版本。
5. 每完成一个阶段功能后应进行独立 Git 提交，避免一次性提交全部成果。