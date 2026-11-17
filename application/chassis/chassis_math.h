#ifndef CHASSIS_MATH_H
#define CHASSIS_MATH_H

#include <math.h>

static inline void ChassisLimitVector(float *x, float *y, float limit)
{
    float norm = hypotf(*x, *y);
    if (norm > limit)
    {
        *x *= limit / norm;
        *y *= limit / norm;
    }
}

static inline void ChassisSlewVector(float *x, float *y, float target_x, float target_y,
                                    float accel, float decel, float dt)
{
    float dx = target_x - *x, dy = target_y - *y;
    float distance = hypotf(dx, dy);
    float rate = (hypotf(target_x, target_y) < hypotf(*x, *y) ||
                  target_x * *x + target_y * *y < 0.0f) ? decel : accel;
    float step = rate * dt;
    if (distance <= step || distance == 0.0f)
    {
        *x = target_x;
        *y = target_y;
    }
    else
    {
        *x += dx * step / distance;
        *y += dy * step / distance;
    }
}

/* 四轮同比例限幅，保留平移和旋转的混合比例。 */
static inline float ChassisScaleWheels(float wheel[4], float max_rpm)
{
    float peak = 0.0f;
    for (unsigned i = 0; i < 4; ++i)
        peak = fmaxf(peak, fabsf(wheel[i]));
    float scale = peak > max_rpm ? max_rpm / peak : 1.0f;
    for (unsigned i = 0; i < 4; ++i)
        wheel[i] *= scale;
    return scale;
}

/* 混合运动优先保留平移；旋转使用剩余轮速空间，原地旋转不降额。 */
static inline float ChassisMixWheels(const float translation[4], const float rotation[4],
                                    float max_rpm, float translation_share, float wheel[4],
                                    float *translation_scale)
{
    float peak = 0.0f, spin_peak = 0.0f;
    for (unsigned i = 0; i < 4; ++i)
    {
        peak = fmaxf(peak, fabsf(translation[i]));
        spin_peak = fmaxf(spin_peak, fabsf(rotation[i]));
    }
    /* 随旋转斜坡逐步预留空间，避免刚开启自旋就跳降平移。 */
    float cap = max_rpm - fminf(spin_peak, max_rpm * (1.0f - translation_share));
    *translation_scale = peak > cap ? cap / peak : 1.0f;
    float spin_scale = 1.0f;
    for (unsigned i = 0; i < 4; ++i)
    {
        float t = translation[i] * *translation_scale;
        if (rotation[i] > 0.001f)
            spin_scale = fminf(spin_scale, (max_rpm - t) / rotation[i]);
        else if (rotation[i] < -0.001f)
            spin_scale = fminf(spin_scale, (-max_rpm - t) / rotation[i]);
    }
    spin_scale = fmaxf(0.0f, spin_scale);
    for (unsigned i = 0; i < 4; ++i)
        wheel[i] = translation[i] * *translation_scale + rotation[i] * spin_scale;
    return spin_scale;
}

/* 输入为消除电机反向配置后的轮缘速度mm/s，顺序LF、RF、LB、RB。 */
static inline void ChassisInverse(const float wheel[4], const float arm[4],
                                  float *vx, float *vy, float *wz)
{
    *vx = (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) * 0.35355339059f;
    *vy = ( wheel[0] + wheel[1] - wheel[2] - wheel[3]) * 0.35355339059f;
    *wz = (wheel[0] - wheel[1] - wheel[2] + wheel[3]) /
          (arm[0] + arm[1] + arm[2] + arm[3]);
}

#endif
