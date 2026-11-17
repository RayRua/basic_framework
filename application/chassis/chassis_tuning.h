#ifndef CHASSIS_TUNING_H
#define CHASSIS_TUNING_H

/* Sentinel第一阶段：保持旧指令比例和PID，尚不是m/s、deg/s接口。 */
#define CHASSIS_INPUT_MAX                 330000.0f
#define CHASSIS_INPUT_ACCEL               220000.0f  /* 约0.6s到手柄满杆 */
#define CHASSIS_INPUT_DECEL               440000.0f
#define CHASSIS_SPIN_SLEW                  39584.0f  /* 约0.8s到现有自旋目标 */
#define CHASSIS_WHEEL_MAX_RPM               4500.0f  /* 调试初值，需实车确认 */
#define CHASSIS_STICK_DEADBAND                10
#define CHASSIS_DEFAULT_POWER_W              40.0f
#define CHASSIS_MAX_ACCEPTED_POWER_W         200.0f
#define CHASSIS_SPIN_TRANSLATION_SHARE         0.70f
#define CHASSIS_POWER_RISE_WPS                10.0f
#define CHASSIS_BUFFER_FULL_J                 30.0f
#define CHASSIS_BUFFER_MIN_RATIO               0.50f
#define CHASSIS_STATUS_TIMEOUT_MS             1000U
#define CHASSIS_POWER_TIMEOUT_MS               300U

#endif
