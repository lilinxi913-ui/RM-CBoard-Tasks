#include <cstdint>

#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// can_task.cpp 中定义的变量，这里只声明
extern volatile float dbg_a_target;
extern volatile float dbg_a_angle;
extern volatile float dbg_b_target;
extern volatile float dbg_b_angle;
extern volatile float dbg_yaw;

// imu_task.cpp 中定义的 yaw 角速度
extern volatile float linkage_vyaw;

// USART1，使用普通发送
sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  while (true) {
    float a_target;
    float a_angle;
    float b_target;
    float b_angle;
    float yaw;
    float yaw_speed;

    // 快速复制数据，串口发送在临界区外进行
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    a_target = dbg_a_target;
    a_angle = dbg_a_angle;

    b_target = dbg_b_target;
    b_angle = dbg_b_angle;

    // 使用 can_task 展开后的累计 yaw，避免 ±π 跳变
    yaw = dbg_yaw;
    yaw_speed = linkage_vyaw;

    __set_PRIMASK(irq_state);

    plotter.plot(a_target, a_angle, b_target, b_angle, yaw, yaw_speed);

    // 约100Hz输出，不改变电机控制周期
    osDelay(10);
  }
}