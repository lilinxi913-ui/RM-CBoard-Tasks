#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" void buzzer_task(void const * argument)
{
  // 第一声
  buzzer.set(3000, 0.5);
  buzzer.start();

  osDelay(300);

  buzzer.stop();

  osDelay(100);

  // 第二声
  buzzer.set(4000, 0.5);
  buzzer.start();

  osDelay(300);

  buzzer.stop();

  osDelay(100);

  // 第三声
  buzzer.set(5000, 0.5);
  buzzer.start();

  osDelay(300);

  buzzer.stop();

  // 之后保持任务存在
  while (1) {
    osDelay(1000);
  }
}