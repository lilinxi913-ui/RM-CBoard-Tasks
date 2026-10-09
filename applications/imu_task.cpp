#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "task.h"
#include "tools/mahony/mahony.hpp"

const float r_ab[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(0.001f);
volatile float linkage_yaw = 0, linkage_vyaw = 0;
volatile uint32_t linkage_imu_ready = 0, linkage_imu_stamp = 0;
volatile uint32_t dbg_imu_stage = 0;
volatile float dbg_gyro_bias_x = 0, dbg_gyro_bias_y = 0, dbg_gyro_bias_z = 0;

extern "C" void imu_task(void const * argument)
{
  (void)argument;
  bmi088.init();
  // Keep the board stationary throughout warm-up and calibration (~14 s).
  for (int i = 0; i < 1000; ++i) {
    bmi088.update();
    osDelay(10);
  }
  dbg_imu_stage = 1;
  float bias[3] = {0, 0, 0};
  for (int i = 0; i < 2000; ++i) {
    bmi088.update();
    for (int j = 0; j < 3; ++j) bias[j] += bmi088.gyro[j] / 2000.0f;
    osDelay(2);
  }
  dbg_gyro_bias_x = bias[0];
  dbg_gyro_bias_y = bias[1];
  dbg_gyro_bias_z = bias[2];
  TickType_t wake = xTaskGetTickCount();
  uint32_t settling = 0;
  while (true) {
    bmi088.update();
    float gyro[3];
    for (int j = 0; j < 3; ++j) gyro[j] = bmi088.gyro[j] - bias[j];
    imu.update(bmi088.acc, gyro);
    const uint32_t irq = __get_PRIMASK();
    __disable_irq();
    linkage_yaw = imu.yaw;
    linkage_vyaw = imu.vyaw;
    linkage_imu_stamp = osKernelSysTick();
    if (++settling >= 2000U) {
      linkage_imu_ready = 1;
      dbg_imu_stage = 2;
    }
    __set_PRIMASK(irq);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}
