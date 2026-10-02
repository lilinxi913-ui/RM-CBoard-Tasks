#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板使用USART3连接DBUS
sp::DBus remote(&huart3);

extern "C" void uart_task()
{
  // 开启第一次DMA接收
  remote.request();

  while (true) {
    // 后续可以在这里读取遥控器数据
    // 例如：
    // remote.ch[0]

    osDelay(10);
  }
}

// 串口接收完成回调
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  // 判断是不是USART3
  if (huart == &huart3) {
    // 解析遥控器数据
    remote.update(Size, stamp_ms);

    // 开启下一次接收
    remote.request();
  }
}

// UART错误回调
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    // 出错重新接收
    remote.request();
  }
}