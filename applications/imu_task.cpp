#include <cstdio>

#include "cmsis_os.h"
#include "gpio.h"
#include "io/bmi088/bmi088.hpp"
#include "spi.h"

/*
    BMI088坐标系到机器人坐标系的转换矩阵

    当前设置:
    不旋转，保持原始坐标
*/
float r_ab[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

/*
    创建BMI088对象

    SPI:
        SPI1

    CSB1:
        PA4
        加速度计片选

    CSB2:
        PB0
        陀螺仪片选
*/
sp::BMI088 imu(
  &hspi1,

  GPIOA, GPIO_PIN_4,

  GPIOB, GPIO_PIN_0,

  r_ab);

extern "C" void imu_task(void const * argument)
{
  /*
        BMI088初始化

        包括:
        1. 加速度计初始化
        2. 陀螺仪初始化
        3. SPI通信检测
    */
  imu.init();

  while (1) {
    /*
            更新IMU数据

            更新:
            imu.acc[3]
            imu.gyro[3]
            imu.temp
        */
    imu.update();

    /*
            打印加速度数据

            单位:
            m/s^2
        */
    printf("acc: %.3f %.3f %.3f\r\n", imu.acc[0], imu.acc[1], imu.acc[2]);

    /*
            打印角速度数据

            单位:
            rad/s
        */
    printf("gyro: %.3f %.3f %.3f\r\n", imu.gyro[0], imu.gyro[1], imu.gyro[2]);

    /*
            打印温度

            单位:
            摄氏度
        */
    printf("temp: %.2f\r\n", imu.temp);

    /*
            100Hz读取频率

            10ms一次
        */
    osDelay(10);
  }
}