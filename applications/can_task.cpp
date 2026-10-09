#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "main.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"

// uart_task.cpp 中定义的遥控器
extern sp::DBus remote;

// 上次代码包 imu_task.cpp 发布的数据
extern volatile float linkage_yaw;
extern volatile float linkage_vyaw;
extern volatile uint32_t linkage_imu_ready;
extern volatile uint32_t linkage_imu_stamp;

// ==================== CAN 与电机 ====================

sp::CAN can1(&hcan1);

sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

// 实际转动方向与 C 板正 yaw 一致时为 1。
// 某台方向相反时，将对应值改成 -1，再重新烧录、标定。
constexpr float A_SIGN = 1.0f;
constexpr float B_SIGN = 1.0f;

constexpr float PI = 3.14159265358979323846f;
constexpr float DT = 0.001f;

// ==================== PID 参数 ====================

// 位置环沿用你调好的增益
constexpr float POSITION_KP = 5.0f;
constexpr float POSITION_KI = 2.5f;
constexpr float POSITION_KD = 0.0f;

// 位置环输出是目标速度，单位 rad/s
constexpr float MAX_SPEED = 15.0f;
constexpr float MAX_POSITION_IOUT = 2.5f;

// 速度环参数
constexpr float SPEED_KP = 0.035f;
constexpr float SPEED_KI = 0.0f;
constexpr float SPEED_KD = 0.0f;

// 速度环输出是力矩，单位 N·m
constexpr float MAX_TORQUE = 0.3f;

// Ki=0 时积分限幅不参与输出
constexpr float MAX_SPEED_IOUT = 0.2f;

// angular=false：使用累计多圈位置误差
sp::PID a_position_pid(
  DT, POSITION_KP, POSITION_KI, POSITION_KD, MAX_SPEED, MAX_POSITION_IOUT, 1.0f, false, true);

sp::PID b_position_pid(
  DT, POSITION_KP, POSITION_KI, POSITION_KD, MAX_SPEED, MAX_POSITION_IOUT, 1.0f, false, true);

sp::PID a_speed_pid(
  DT, SPEED_KP, SPEED_KI, SPEED_KD, MAX_TORQUE, MAX_SPEED_IOUT, 1.0f, false, true);

sp::PID b_speed_pid(
  DT, SPEED_KP, SPEED_KI, SPEED_KD, MAX_TORQUE, MAX_SPEED_IOUT, 1.0f, false, true);

// ==================== 调试变量 ====================

// 右下档、R 标手动对齐后，设置为 1，记录标定。
// 标定保存在 RAM，复位、断电后需重新标定。
volatile uint32_t dbg_calibrate = 0;
volatile uint32_t dbg_calibrated = 0;

// 手动识别默认开启，无需调试命令。
// A拖动判定力矩乘速度阈值为-0.001，B为-0.003。
volatile uint32_t dbg_manual_enable = 1;

// 0：失能；1：联动；2：复位
volatile uint32_t dbg_mode = 0;

// 0：无手动接管；1：A；2：B
volatile uint32_t dbg_manual_leader = 0;

volatile uint32_t dbg_a_alive = 0;
volatile uint32_t dbg_b_alive = 0;
volatile uint32_t dbg_remote_alive = 0;

volatile uint32_t dbg_a_rx_count = 0;
volatile uint32_t dbg_b_rx_count = 0;
volatile uint32_t dbg_task_count = 0;

volatile uint32_t dbg_a_enabled = 0;
volatile uint32_t dbg_sw_r = 0;
volatile uint32_t dbg_sw_l = 0;

volatile float dbg_a_angle = 0.0f;
volatile float dbg_b_angle = 0.0f;

volatile float dbg_a_target = 0.0f;
volatile float dbg_b_target = 0.0f;

volatile float dbg_a_speed = 0.0f;
volatile float dbg_b_speed = 0.0f;

volatile float dbg_a_speed_target = 0.0f;
volatile float dbg_b_speed_target = 0.0f;

volatile float dbg_a_torque_cmd = 0.0f;
volatile float dbg_b_torque_cmd = 0.0f;

volatile float dbg_ratio = 0.5f;
volatile float dbg_yaw = 0.0f;
volatile float dbg_yaw_reference = 0.0f;

// 兼容之前速度波形使用的变量
volatile float dbg_target = 0.0f;
volatile float dbg_speed = 0.0f;
volatile float dbg_output = 0.0f;
volatile float dbg_a_iout = 0.0f;

// ==================== 工具函数 ====================

static float wrap_angle(float angle)
{
  while (angle > PI) {
    angle -= 2.0f * PI;
  }

  while (angle < -PI) {
    angle += 2.0f * PI;
  }

  return angle;
}

// 找到与目标方向相同、距离当前位置最近的角度
static float nearest_angle(float target, float actual)
{
  return actual + wrap_angle(target - actual);
}

static void clear_all_pid()
{
  a_position_pid.clear();
  b_position_pid.clear();

  a_speed_pid.clear();
  b_speed_pid.clear();
}

static float get_ratio(sp::DBusSwitchMode sw)
{
  if (sw == sp::DBusSwitchMode::DOWN) {
    return 0.5f;
  }

  if (sw == sp::DBusSwitchMode::MID) {
    return -1.0f;
  }

  return 3.0f;
}

static bool switch_valid(sp::DBusSwitchMode sw)
{
  return sw == sp::DBusSwitchMode::DOWN || sw == sp::DBusSwitchMode::MID ||
         sw == sp::DBusSwitchMode::UP;
}

// ==================== 控制任务 ====================

extern "C" void can_task(void const * argument)
{
  (void)argument;

  can1.config();
  can1.start();

  TickType_t wake = xTaskGetTickCount();

  bool armed = false;
  bool yaw_initialized = false;

  int previous_mode = 0;
  int manual_leader = 0;
  int manual_candidate = 0;

  float yaw = 0.0f;
  float last_raw_yaw = 0.0f;
  float last_yaw = 0.0f;

  // R 标对齐时的标定角度
  float calibrated_a = 0.0f;
  float calibrated_b = 0.0f;
  float calibrated_yaw = 0.0f;

  // 联动参考
  float anchor_a = 0.0f;
  float anchor_b = 0.0f;
  float linkage_position = 0.0f;
  float ratio = 0.5f;

  float target_a = 0.0f;
  float target_b = 0.0f;

  // 上一周期的命令力矩，用于手动拖动判定
  float torque_a = 0.0f;
  float torque_b = 0.0f;

  uint32_t board_still_since = 0;
  uint32_t candidate_since = 0;
  uint32_t release_since = 0;
  uint32_t key_press_start = 0;
  bool key_pressed = false;
  bool key_triggered = false;
  bool key_blocked = true;

  while (true) {
    ++dbg_task_count;

    // ---------- 读取一致的输入快照 ----------

    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    const uint32_t now = osKernelSysTick();

    const bool a_alive = motor_a.is_alive(now);
    const bool b_alive = motor_b.is_alive(now);
    const bool remote_alive = remote.is_alive(now);

    const float angle_a = A_SIGN * motor_a.angle;
    const float angle_b = B_SIGN * motor_b.angle;

    const float speed_a = A_SIGN * motor_a.speed;
    const float speed_b = B_SIGN * motor_b.speed;

    const float raw_yaw = linkage_yaw;
    const float yaw_speed = linkage_vyaw;

    const bool imu_ok = linkage_imu_ready != 0U && (now - linkage_imu_stamp < 20U);

    const auto sw_r = remote.sw_r;
    const auto sw_l = remote.sw_l;

    __set_PRIMASK(irq_state);

    // ---------- yaw 多圈展开 ----------

    if (imu_ok) {
      if (!yaw_initialized) {
        yaw = raw_yaw;
        last_raw_yaw = raw_yaw;
        yaw_initialized = true;
      }
      else {
        yaw += wrap_angle(raw_yaw - last_raw_yaw);
        last_raw_yaw = raw_yaw;
      }
    }

    const bool inputs_ok =
      a_alive && b_alive && remote_alive && imu_ok && switch_valid(sw_r) && switch_valid(sw_l);

    // 掉线后必须重新回右下档才能使能
    if (!inputs_ok) {
      armed = false;
    }
    else if (sw_r == sp::DBusSwitchMode::DOWN) {
      armed = true;
    }

    // ---------- 右下档长按KEY两秒标定，保存在RAM ----------
    const bool key_down = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;
    const bool allow_key = inputs_ok && sw_r == sp::DBusSwitchMode::DOWN;
    if (!allow_key) {
      key_pressed = false;
      key_triggered = false;
      key_blocked = true;
    }
    else if (!key_down) {
      key_pressed = false;
      key_triggered = false;
      key_blocked = false;
    }
    else if (!key_blocked) {
      if (!key_pressed) {
        key_pressed = true;
        key_press_start = now;
      }
      if (!key_triggered && now - key_press_start >= 2000U) {
        dbg_calibrate = 1;
        key_triggered = true;
      }
    }

    // ---------- R 标标定 ----------

    if (dbg_calibrate != 0U) {
      if (inputs_ok && sw_r == sp::DBusSwitchMode::DOWN) {
        calibrated_a = angle_a;
        calibrated_b = angle_b;
        calibrated_yaw = yaw;

        dbg_calibrated = 1;
      }

      dbg_calibrate = 0;
    }

    // ---------- 模式选择 ----------

    int mode = 0;

    if (inputs_ok && armed && dbg_calibrated != 0U) {
      if (sw_r == sp::DBusSwitchMode::MID) {
        mode = 1;
      }
      else if (sw_r == sp::DBusSwitchMode::UP) {
        mode = 2;
      }
    }

    const float new_ratio = get_ratio(sw_l);

    const bool entering_mode = mode != previous_mode;

    const bool changing_ratio = mode == 1 && new_ratio != ratio;

    // 进入模式或切换比例时从实际位置建立新参考
    if (entering_mode || changing_ratio) {
      clear_all_pid();

      manual_leader = 0;
      manual_candidate = 0;

      candidate_since = now;
      release_since = now;
      board_still_since = now;

      anchor_a = angle_a;
      anchor_b = angle_b;

      linkage_position = 0.0f;

      target_a = angle_a;
      target_b = angle_b;
    }

    ratio = new_ratio;

    if (std::fabs(yaw_speed) > 0.05f) {
      board_still_since = now;
    }

    // ---------- 右中档：联动 ----------

    if (mode == 1) {
      if (!entering_mode && !changing_ratio) {
        linkage_position += yaw - last_yaw;
      }

      target_a = anchor_a + linkage_position;
      target_b = anchor_b + ratio * linkage_position;

      // 关闭手动识别时清理接管状态
      if (dbg_manual_enable == 0U && manual_leader != 0) {
        manual_leader = 0;
        manual_candidate = 0;

        clear_all_pid();
      }

      // 手动拖动判定：
      // C板静止，电机有位移误差，且运动方向与命令力矩相反。
      // 这是经验判定，需要实机调整阈值。
      if (manual_leader == 0 && dbg_manual_enable != 0U && now - board_still_since > 300U) {
        const bool dragging_a = std::fabs(speed_a) > 0.10f &&
                                std::fabs(target_a - angle_a) > 0.04f &&
                                torque_a * speed_a < -0.001f;

        const bool dragging_b = std::fabs(speed_b) > 0.10f &&
                                std::fabs(target_b - angle_b) > 0.04f &&
                                torque_b * speed_b < -0.003f;

        const int detected = dragging_a ? 1 : (dragging_b ? 2 : 0);

        if (detected != manual_candidate) {
          manual_candidate = detected;
          candidate_since = now;
        }

        if (manual_candidate != 0 && now - candidate_since >= 80U) {
          manual_leader = manual_candidate;
          release_since = now;

          clear_all_pid();
        }
      }
      else if (manual_leader == 0) {
        manual_candidate = 0;
      }

      // 被拖动的一台作为位置输入，另一台跟随
      if (manual_leader != 0) {
        if (manual_leader == 1) {
          linkage_position = angle_a - anchor_a;
        }
        else {
          linkage_position = (angle_b - anchor_b) / ratio;
        }

        target_a = anchor_a + linkage_position;
        target_b = anchor_b + ratio * linkage_position;

        const float manual_speed = manual_leader == 1 ? speed_a : speed_b;

        if (std::fabs(manual_speed) > 0.08f) {
          release_since = now;
        }

        // 停止拖动300ms后恢复双电机位置控制
        if (now - release_since > 300U) {
          manual_leader = 0;
          manual_candidate = 0;

          clear_all_pid();
        }
      }
    }

    // ---------- 右上档：R 标方向复位 ----------

    else if (mode == 2) {
      // 假设两台电机底座固定、C板独立转动。
      // 目标是与C板当前R方向一致，选择最近路径。
      target_a = nearest_angle(calibrated_a + yaw - calibrated_yaw, angle_a);

      target_b = nearest_angle(calibrated_b + yaw - calibrated_yaw, angle_b);
    }

    // ---------- 失能 ----------

    else {
      target_a = angle_a;
      target_b = angle_b;

      manual_leader = 0;
      manual_candidate = 0;

      clear_all_pid();
    }

    // ---------- 位置环 → 速度环 → 力矩 ----------

    torque_a = 0.0f;
    torque_b = 0.0f;

    float speed_target_a = 0.0f;
    float speed_target_b = 0.0f;

    if (mode != 0) {
      if (manual_leader != 1) {
        a_position_pid.calc(target_a, angle_a);
        speed_target_a = a_position_pid.out;

        a_speed_pid.calc(speed_target_a, speed_a);
        torque_a = a_speed_pid.out;
      }
      else {
        // 手动输入端撤去力矩
        a_position_pid.clear();
        a_speed_pid.clear();
      }

      if (manual_leader != 2) {
        b_position_pid.calc(target_b, angle_b);
        speed_target_b = b_position_pid.out;

        b_speed_pid.calc(speed_target_b, speed_b);
        torque_b = b_speed_pid.out;
      }
      else {
        b_position_pid.clear();
        b_speed_pid.clear();
      }
    }

    // ---------- CAN发送 ----------

    motor_a.cmd(A_SIGN * torque_a);
    motor_b.cmd(B_SIGN * torque_b);

    for (auto & byte : can1.tx_data) {
      byte = 0;
    }

    motor_a.write(can1.tx_data);
    motor_b.write(can1.tx_data);

    // ID1、ID2处于同一个控制报文，发送一次即可
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
      can1.send(motor_a.tx_id);
    }

    // ---------- 调试数据 ----------

    dbg_a_alive = a_alive ? 1U : 0U;
    dbg_b_alive = b_alive ? 1U : 0U;
    dbg_remote_alive = remote_alive ? 1U : 0U;

    dbg_sw_r = static_cast<uint32_t>(sw_r);
    dbg_sw_l = static_cast<uint32_t>(sw_l);

    dbg_mode = static_cast<uint32_t>(mode);
    dbg_manual_leader = static_cast<uint32_t>(manual_leader);

    dbg_a_enabled = mode != 0 && manual_leader != 1 ? 1U : 0U;

    dbg_a_angle = angle_a;
    dbg_b_angle = angle_b;

    dbg_a_target = target_a;
    dbg_b_target = target_b;

    dbg_a_speed = speed_a;
    dbg_b_speed = speed_b;

    dbg_a_speed_target = speed_target_a;
    dbg_b_speed_target = speed_target_b;

    dbg_a_torque_cmd = torque_a;
    dbg_b_torque_cmd = torque_b;

    dbg_ratio = ratio;
    dbg_yaw = yaw;

    // 只更新虚拟参考，不修改IMU实际yaw
    dbg_yaw_reference = yaw - linkage_position;

    dbg_target = speed_target_a;
    dbg_speed = speed_a;
    dbg_output = torque_a;
    dbg_a_iout = a_speed_pid.data.iout;

    last_yaw = yaw;
    previous_mode = mode;

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

// ==================== CAN接收回调 ====================

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    can1.recv();

    // 只处理标准数据帧
    if (can1.frame_type) {
      continue;
    }

    const uint32_t stamp = osKernelSysTick();

    if (can1.rx_id == motor_a.rx_id) {
      motor_a.read(can1.rx_data, stamp);
      ++dbg_a_rx_count;
    }
    else if (can1.rx_id == motor_b.rx_id) {
      motor_b.read(can1.rx_data, stamp);
      ++dbg_b_rx_count;
    }
  }
}
