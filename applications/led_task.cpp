#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const * argument)
{
  led.start();

  while (1) {
    // 红灯
    led.set(1.0f, 0.0f, 0.0f);
    osDelay(300);

    // 绿灯
    led.set(0.0f, 1.0f, 0.0f);
    osDelay(300);

    // 蓝灯
    led.set(0.0f, 0.0f, 1.0f);
    osDelay(300);
  }
}