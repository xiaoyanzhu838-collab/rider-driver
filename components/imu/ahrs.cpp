#include "ahrs.h"

#include <cmath>
#include <cstring>
#include <new>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "board_config.h"
#include "third_party/vqf/vqf.hpp"

namespace {

constexpr float kDegToRad = 0.01745329252f;
constexpr float kRadToDeg = 57.29577951f;
constexpr float kGravity = 9.80665f;
constexpr double kHalfPi = 1.57079632679489661923;

static const char *TAG = "ahrs";
static portMUX_TYPE s_ahrs_mux = portMUX_INITIALIZER_UNLOCKED;
static VQF *s_vqf = nullptr;
static ahrs_state_t s_state = {{0.0f, 0.0f, 0.0f},
                               {1.0f, 0.0f, 0.0f, 0.0f},
                               {0.0f, 0.0f, 1.0f},
                               0.0f,
                               {0.0f, 0.0f, 0.0f}};
static float s_sample_period_s = 0.005f;

static void quat_to_euler(const vqf_real_t q[4], ahrs_euler_t *out)
{
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];

    const double sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
    const double cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    out->roll = (float)(std::atan2(sinr_cosp, cosr_cosp) * kRadToDeg);

    const double sinp = 2.0 * (q0 * q2 - q3 * q1);
    if (std::fabs(sinp) >= 1.0) {
        out->pitch = (float)(std::copysign(kHalfPi, sinp) * kRadToDeg);
    } else {
        out->pitch = (float)(std::asin(sinp) * kRadToDeg);
    }

    const double siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
    const double cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    out->yaw = (float)(std::atan2(siny_cosp, cosy_cosp) * kRadToDeg);
}

static void refresh_cached_state_locked()
{
    if (!s_vqf) {
        return;
    }

    vqf_real_t quat[4] = {1.0, 0.0, 0.0, 0.0};
    vqf_real_t quat_conj[4] = {1.0, 0.0, 0.0, 0.0};
    vqf_real_t bias[3] = {0.0, 0.0, 0.0};
    vqf_real_t gravity_earth[3] = {0.0, 0.0, 1.0};
    vqf_real_t gravity_body[3] = {0.0, 0.0, 1.0};

    s_vqf->getQuat6D(quat);
    s_vqf->getBiasEstimate(bias);
    VQF::quatConj(quat, quat_conj);
    VQF::quatRotate(quat_conj, gravity_earth, gravity_body);

    for (int i = 0; i < 4; i++) {
        s_state.quat[i] = (float)quat[i];
    }
    for (int i = 0; i < 3; i++) {
        s_state.gravity_body[i] = (float)gravity_body[i];
        s_state.bias_dps[i] = (float)(bias[i] * kRadToDeg);
    }

    s_state.theta_fb_deg =
        (float)(std::atan2(-gravity_body[1], gravity_body[2]) * kRadToDeg);
    quat_to_euler(quat, &s_state.euler);
}

}  // namespace

extern "C" {

void ahrs_init(float sample_freq_hz)
{
    portENTER_CRITICAL(&s_ahrs_mux);

    if (sample_freq_hz > 0.0f) {
        s_sample_period_s = 1.0f / sample_freq_hz;
    }

    delete s_vqf;
    s_vqf = nullptr;

    VQFParams params;
    params.tauAcc = BOARD_VQF_TAU_ACC;
    params.magDistRejectionEnabled = false;
    s_vqf = new (std::nothrow) VQF(params, (vqf_real_t)s_sample_period_s);

    s_state.euler = {0.0f, 0.0f, 0.0f};
    s_state.quat[0] = 1.0f;
    s_state.quat[1] = 0.0f;
    s_state.quat[2] = 0.0f;
    s_state.quat[3] = 0.0f;
    s_state.gravity_body[0] = 0.0f;
    s_state.gravity_body[1] = 0.0f;
    s_state.gravity_body[2] = 1.0f;
    s_state.theta_fb_deg = 0.0f;
    s_state.bias_dps[0] = 0.0f;
    s_state.bias_dps[1] = 0.0f;
    s_state.bias_dps[2] = 0.0f;

    if (s_vqf) {
        refresh_cached_state_locked();
    }

    portEXIT_CRITICAL(&s_ahrs_mux);

    ESP_LOGI(TAG,
             "using VQF 6D: Ts=%.4fs tauAcc=%.2fs",
             (double)s_sample_period_s,
             (double)BOARD_VQF_TAU_ACC);
}

void ahrs_reset(void)
{
    portENTER_CRITICAL(&s_ahrs_mux);
    if (s_vqf) {
        s_vqf->resetState();
        refresh_cached_state_locked();
    }
    portEXIT_CRITICAL(&s_ahrs_mux);
}

void ahrs_update(float ax, float ay, float az, float gx, float gy, float gz)
{
    vqf_real_t gyr[3] = {
        (vqf_real_t)(gx * kDegToRad),
        (vqf_real_t)(gy * kDegToRad),
        (vqf_real_t)(gz * kDegToRad),
    };
    vqf_real_t acc[3] = {
        (vqf_real_t)(ax * kGravity),
        (vqf_real_t)(ay * kGravity),
        (vqf_real_t)(az * kGravity),
    };

    portENTER_CRITICAL(&s_ahrs_mux);
    if (s_vqf) {
        s_vqf->update(gyr, acc);
        refresh_cached_state_locked();
    }
    portEXIT_CRITICAL(&s_ahrs_mux);
}

void ahrs_get_euler(ahrs_euler_t *out)
{
    if (!out) {
        return;
    }

    portENTER_CRITICAL(&s_ahrs_mux);
    *out = s_state.euler;
    portEXIT_CRITICAL(&s_ahrs_mux);
}

void ahrs_get_state(ahrs_state_t *out)
{
    if (!out) {
        return;
    }

    portENTER_CRITICAL(&s_ahrs_mux);
    *out = s_state;
    portEXIT_CRITICAL(&s_ahrs_mux);
}

}  // extern "C"
