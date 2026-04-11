#include "ahrs.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// ============================================================
// Madgwick AHRS 滤波器（6-axis, 无磁力计）
// 参考: Sebastian Madgwick, "An efficient orientation filter
//        for inertial and inertial/magnetic sensor arrays"
// ============================================================

#define DEG2RAD (0.01745329252f)  // PI / 180
#define RAD2DEG (57.29577951f)    // 180 / PI

// 滤波增益（beta），越大加速度计权重越高，收敛越快但越敏感噪声
// 典型值 0.01~0.1，机器人场景取 0.04
#define MADGWICK_BETA  0.04f

static portMUX_TYPE s_ahrs_mux = portMUX_INITIALIZER_UNLOCKED;

// 四元数
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 采样频率的倒数
static float s_inv_sample_freq = 0.005f;  // 默认 200 Hz

void ahrs_init(float sample_freq_hz)
{
    portENTER_CRITICAL(&s_ahrs_mux);
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    if (sample_freq_hz > 0.0f) {
        s_inv_sample_freq = 1.0f / sample_freq_hz;
    }
    portEXIT_CRITICAL(&s_ahrs_mux);
}

void ahrs_reset(void)
{
    portENTER_CRITICAL(&s_ahrs_mux);
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    portEXIT_CRITICAL(&s_ahrs_mux);
}

static float inv_sqrt(float x)
{
    // 用标准库，编译器会优化
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

void ahrs_update(float ax, float ay, float az, float gx, float gy, float gz)
{
    // 陀螺仪 dps → rad/s
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    portENTER_CRITICAL(&s_ahrs_mux);

    float _q0 = q0, _q1 = q1, _q2 = q2, _q3 = q3;

    // 四元数微分（纯陀螺仪积分）
    float qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    float qDot1 = 0.5f * ( _q0 * gx + _q2 * gz - _q3 * gy);
    float qDot2 = 0.5f * ( _q0 * gy - _q1 * gz + _q3 * gx);
    float qDot3 = 0.5f * ( _q0 * gz + _q1 * gy - _q2 * gx);

    // 加速度计校正（only if valid data）
    float a_norm_sq = ax * ax + ay * ay + az * az;
    if (a_norm_sq > 0.01f) {
        float recip_norm = inv_sqrt(a_norm_sq);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // 梯度下降：目标函数 f = estimated_gravity - measured_gravity
        // 其中 estimated_gravity 从四元数推导
        float _2q0 = 2.0f * _q0;
        float _2q1 = 2.0f * _q1;
        float _2q2 = 2.0f * _q2;
        float _2q3 = 2.0f * _q3;
        float _4q0 = 4.0f * _q0;
        float _4q1 = 4.0f * _q1;
        float _4q2 = 4.0f * _q2;
        float _8q1 = 8.0f * _q1;
        float _8q2 = 8.0f * _q2;
        float q0q0 = _q0 * _q0;
        float q1q1 = _q1 * _q1;
        float q2q2 = _q2 * _q2;
        float q3q3 = _q3 * _q3;

        // 梯度
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * _q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;

        // 归一化梯度
        recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        // 应用反馈
        qDot0 -= MADGWICK_BETA * s0;
        qDot1 -= MADGWICK_BETA * s1;
        qDot2 -= MADGWICK_BETA * s2;
        qDot3 -= MADGWICK_BETA * s3;
    }

    // 积分
    _q0 += qDot0 * s_inv_sample_freq;
    _q1 += qDot1 * s_inv_sample_freq;
    _q2 += qDot2 * s_inv_sample_freq;
    _q3 += qDot3 * s_inv_sample_freq;

    // 归一化四元数
    float recip_norm = inv_sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    q0 = _q0 * recip_norm;
    q1 = _q1 * recip_norm;
    q2 = _q2 * recip_norm;
    q3 = _q3 * recip_norm;

    portEXIT_CRITICAL(&s_ahrs_mux);
}

void ahrs_get_euler(ahrs_euler_t *out)
{
    if (!out) return;

    portENTER_CRITICAL(&s_ahrs_mux);
    float _q0 = q0, _q1 = q1, _q2 = q2, _q3 = q3;
    portEXIT_CRITICAL(&s_ahrs_mux);

    // 四元数 → 欧拉角 (ZYX convention)
    // roll  (X)
    float sinr_cosp = 2.0f * (_q0 * _q1 + _q2 * _q3);
    float cosr_cosp = 1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2);
    out->roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

    // pitch (Y)
    float sinp = 2.0f * (_q0 * _q2 - _q3 * _q1);
    if (fabsf(sinp) >= 1.0f) {
        out->pitch = copysignf(90.0f, sinp);  // gimbal lock
    } else {
        out->pitch = asinf(sinp) * RAD2DEG;
    }

    // yaw   (Z)
    float siny_cosp = 2.0f * (_q0 * _q3 + _q1 * _q2);
    float cosy_cosp = 1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3);
    out->yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}
