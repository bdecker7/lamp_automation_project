
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <math.h>  // for fabsf

// ===== Configuration =====
#define TAG                     "SERVO_BTN"

// --- Servo config ---
#define SERVO_GPIO              (GPIO_NUM_1)          // Consider GPIO18 if UART TX on GPIO1 causes issues
#define SERVO_LEDC_MODE         (LEDC_LOW_SPEED_MODE)
#define SERVO_LEDC_TIMER        (LEDC_TIMER_0)
#define SERVO_LEDC_CHANNEL      (LEDC_CHANNEL_0)
#define SERVO_PWM_FREQ_HZ       (50)                   // 50 Hz → 20,000 µs period
#define SERVO_PWM_RES           (LEDC_TIMER_14_BIT)    // If this errors, drop to 13 or 12 bit
#define SERVO_MAX_DUTY          ((1 << 14) - 1)        // 16383 @ 14-bit

// Angle mapping range
#define SERVO_MIN_ANGLE_DEG     (0.0f)
#define SERVO_MAX_ANGLE_DEG     (180.0f)

#define SERVO_SAFE_MIN_US       (1000)
#define SERVO_SAFE_MID_US       (1450)
#define SERVO_SAFE_MAX_US       (1800)

// Current endpoints that map to angles:
static float pulse_min_us = SERVO_SAFE_MIN_US;
static float pulse_mid_us = SERVO_SAFE_MID_US;
static float pulse_max_us = SERVO_SAFE_MAX_US;

// --- Button config (active-low recommended) ---
#define BUTTON_GPIO             (GPIO_NUM_4)
#define BUTTON_ACTIVE_LEVEL     (0)
#define BUTTON_DEBOUNCE_MS      (500)

// ===== Globals =====
static volatile bool g_trigger_cycle = false;
static volatile int64_t g_last_press_us = 0;

// ===== Servo helpers =====

static inline uint32_t pulsewidth_us_to_duty(float pulse_us)
{
    const float period_us = 1000000.0f / SERVO_PWM_FREQ_HZ;  // 50 Hz -> 20000 us
    float duty_f = (pulse_us / period_us) * (float)SERVO_MAX_DUTY;
    if (duty_f < 0) duty_f = 0;
    if (duty_f > SERVO_MAX_DUTY) duty_f = SERVO_MAX_DUTY;
    return (uint32_t)duty_f;
}

static inline esp_err_t servo_set_pulse_us(float pulse_us)
{
    uint32_t duty = pulsewidth_us_to_duty(pulse_us);
    esp_err_t err = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

// Map 0..180° piecewise to safe endpoints: 0..90 → min..mid, 90..180 → mid..max
static inline float angle_to_pulse_us_safe(float deg)
{
    // Clamp angle
    if (deg < SERVO_MIN_ANGLE_DEG) deg = SERVO_MIN_ANGLE_DEG;
    if (deg > SERVO_MAX_ANGLE_DEG) deg = SERVO_MAX_ANGLE_DEG;

    if (deg <= 90.0f) {
        // 0..90 → min..mid
        const float t = deg / 90.0f;
        return pulse_min_us + t * (pulse_mid_us - pulse_min_us);
    } else {
        // 90..180 → mid..max
        const float t = (deg - 90.0f) / 90.0f;
        return pulse_mid_us + t * (pulse_max_us - pulse_mid_us);
    }
}
// Sets the angle
static inline esp_err_t servo_set_angle(float deg)
{
    float pulse_us = angle_to_pulse_us_safe(deg);
    return servo_set_pulse_us(pulse_us);
}

//function for small servo that has angle assignment to press lamp button
static void small_motor_move(void)
{
    servo_set_angle(0);                            //Starts with servo arm up
    vTaskDelay(pdMS_TO_TICKS(1000));            //delay for 1 second
    servo_set_angle(180);                       //Servo arm goes down
    vTaskDelay(pdMS_TO_TICKS(1000));            // delay for 1 second
    servo_set_angle(0);                         //Servo arm goes up again and stays

}


// ===== Button ISR & setup =====
static void IRAM_ATTR button_isr(void *arg)
{
    // NEGEDGE interrupt—only runs on press for active-low wiring
    int64_t now_us = esp_timer_get_time();
    if (now_us - g_last_press_us > (int64_t)BUTTON_DEBOUNCE_MS * 1000) {
        g_last_press_us = now_us;
        g_trigger_cycle = true;
    }
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask   = (1ULL << BUTTON_GPIO),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = (BUTTON_ACTIVE_LEVEL == 0) ? GPIO_PULLUP_ENABLE  : GPIO_PULLUP_DISABLE,
        .pull_down_en   = (BUTTON_ACTIVE_LEVEL == 1) ? GPIO_PULLDOWN_ENABLE: GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL));
}

void app_main(void)
{
    // LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode       = SERVO_LEDC_MODE,
        .timer_num        = SERVO_LEDC_TIMER,
        .duty_resolution  = SERVO_PWM_RES,
        .freq_hz          = SERVO_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // LEDC channel
    ledc_channel_config_t ch_conf = {
        .gpio_num       = SERVO_GPIO,
        .speed_mode     = SERVO_LEDC_MODE,
        .channel        = SERVO_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = SERVO_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    // Button
    button_init();

    // When button is pressed, g_trigger_cycle returns true and performs the rotation for the servo
    // If button is not pressed, then it is stopped and does nothing.
    while (1) {
        if (g_trigger_cycle) {
            g_trigger_cycle = false;   // consume event
            // perform_cycle();
            small_motor_move();
            vTaskDelay(pdMS_TO_TICKS(100)); // small post-cycle delay
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

