#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"

// bmi088 在 imu_task.cpp 中定义
extern sp::BMI088 bmi088;

// 使用 USART1
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(bmi088.acc[0], bmi088.acc[1], bmi088.acc[2]);

    osDelay(10);
  }
}