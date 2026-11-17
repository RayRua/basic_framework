#ifndef POWER_LIMIT_MATH_H
#define POWER_LIMIT_MATH_H
#include <math.h>
#include <stdint.h>

static inline int PowerDataFresh(uint32_t now, uint32_t stamp, int seen, uint32_t timeout)
{
    return seen && (uint32_t)(now - stamp) <= timeout;
}

/* 缓冲只降额，不借能量提高上限；恢复缓升，降低立即生效。 */
static inline float PowerBudget(float base, float buffer, int buffer_fresh,
                                float full_buffer, float min_ratio)
{
    if (!buffer_fresh || !isfinite(buffer) || buffer < 0.0f)
        return base;
    float fraction = fminf(1.0f, buffer / full_buffer);
    return base * (min_ratio + (1.0f - min_ratio) * fraction);
}

static inline float PowerBudgetSlew(float previous, float target, float rise_wps, float dt)
{
    return target < previous ? target : fminf(target, previous + rise_wps * dt);
}

/* 限制到目标功率可实现的电流；不放大原请求、不改变方向。 */
static inline float PowerLimitCurrent(float request, float a, float b, float c)
{
    float discriminant = b * b - 4.0f * a * c;
    if (!isfinite(request) || !isfinite(discriminant) || a <= 0.0f || discriminant < 0.0f)
        return 0.0f;
    if (request > 0.0f)
        return fminf(request, fmaxf(0.0f, (-b + sqrtf(discriminant)) / (2.0f * a)));
    if (request < 0.0f)
        return fmaxf(request, fminf(0.0f, (-b - sqrtf(discriminant)) / (2.0f * a)));
    return 0.0f;
}
#endif
