
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

// ===== Continuous Rotation Servo Parameters =====
#define CR_NEUTRAL_US           (1500)     // Adjust until the servo truly stops
#define CR_DEADBAND_US          (12)       // Band around neutral forced to stop
#define CR_MAX_FWD_US           (1800)     // Forward upper bound (increase only if safe)
#define CR_MAX_REV_US           (1200)     // Reverse lower bound (decrease only if safe)

// Motion timing/shape
#define RAMP_STEPS              (25)
#define RAMP_MS_PER_STEP        (15)
#define HOLD_MS                 (150)      // (kept for backward compat if used)
#define TARGET_SPEED            (-0.7f)    // (unused now—use FORWARD/REVERSE below)

// --- NEW: Bidirectional speeds/holds ---
#define FORWARD_SPEED           (+0.65f)    //speed of the forward motion (seems like this is a bit faster so I made it less than reverse speed)
#define REVERSE_SPEED           (-0.7f)     //speed of the reverse motion
#define FORWARD_HOLD_MS         (200)       //time the forward motion is on
#define REVERSE_HOLD_MS         (200)       //time the reverse motion is on
#define NEUTRAL_SETTLE_MS       (1000)      //hold time in between the forward and backward motion



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
    // Apply deadband: near neutral → force exact neutral
    if (fabsf(pulse_us - (float)CR_NEUTRAL_US) <= (float)CR_DEADBAND_US) {
        pulse_us = (float)CR_NEUTRAL_US;
    }

    uint32_t duty = pulsewidth_us_to_duty(pulse_us);
    esp_err_t err = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

// Map speed in [-1.0 .. +1.0] to pulse width around neutral
static inline float speed_to_pulse_us(float speed)
{
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    if (speed > 0.0f) {
        // forward: neutral..max_fwd
        return (float)CR_NEUTRAL_US + speed * (float)(CR_MAX_FWD_US - CR_NEUTRAL_US);
    } else if (speed < 0.0f) {
        // reverse: neutral..max_rev
        return (float)CR_NEUTRAL_US + speed * (float)(CR_NEUTRAL_US - CR_MAX_REV_US);
    } else {
        return (float)CR_NEUTRAL_US;
    }
}

static inline esp_err_t servo_set_speed(float speed)
{
    float pulse_us = speed_to_pulse_us(speed);
    return servo_set_pulse_us(pulse_us);
}

// Smooth speed ramp (ease-in-out)
static void servo_ramp_speed(float start_speed, float end_speed, int steps, int ms_per_step)
{
    for (int i = 0; i <= steps; ++i) {
        float t = (float)i / (float)steps;  // 0..1
        float u = (t < 0.5f) ? 2.0f * t * t : -1.0f + (4.0f - 2.0f * t) * t;
        float s = start_speed + u * (end_speed - start_speed);
        (void)servo_set_speed(s);
        vTaskDelay(pdMS_TO_TICKS(ms_per_step));
    }
}

// ===== Motion cycle: Forward -> Stop -> Reverse -> Stop ===== (for continuous motion servo)
static void perform_cycle(void)
{
    const float START = 0.0f;

    // Forward segment
    ESP_LOGI(TAG, "CR: stop -> forward(%.2f) -> hold -> stop", FORWARD_SPEED);
    servo_ramp_speed(START, FORWARD_SPEED, RAMP_STEPS, RAMP_MS_PER_STEP);
    vTaskDelay(pdMS_TO_TICKS(FORWARD_HOLD_MS));
    servo_ramp_speed(FORWARD_SPEED, START, RAMP_STEPS, RAMP_MS_PER_STEP);

    (void)servo_set_speed(0.0f);
    vTaskDelay(pdMS_TO_TICKS(NEUTRAL_SETTLE_MS));

    // Reverse segment
    ESP_LOGI(TAG, "CR: stop -> reverse(%.2f) -> hold -> stop", REVERSE_SPEED);
    servo_ramp_speed(START, REVERSE_SPEED, RAMP_STEPS, RAMP_MS_PER_STEP);
    vTaskDelay(pdMS_TO_TICKS(REVERSE_HOLD_MS));
    servo_ramp_speed(REVERSE_SPEED, START, RAMP_STEPS, RAMP_MS_PER_STEP);

    (void)servo_set_speed(0.0f);  // final stop
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

// void app_main(void)
// {
//     // LEDC timer
//     ledc_timer_config_t timer_conf = {
//         .speed_mode       = SERVO_LEDC_MODE,
//         .timer_num        = SERVO_LEDC_TIMER,
//         .duty_resolution  = SERVO_PWM_RES,
//         .freq_hz          = SERVO_PWM_FREQ_HZ,
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

//     // LEDC channel
//     ledc_channel_config_t ch_conf = {
//         .gpio_num       = SERVO_GPIO,
//         .speed_mode     = SERVO_LEDC_MODE,
//         .channel        = SERVO_LEDC_CHANNEL,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .timer_sel      = SERVO_LEDC_TIMER,
//         .duty           = 0,
//         .hpoint         = 0
//     };
//     ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

//     // Button
//     button_init();

//     // Start stopped at neutral
//     ESP_ERROR_CHECK(servo_set_speed(0.0f));
//     ESP_LOGI(TAG, "Ready. Press button for forward then reverse cycle (CR servo).");


//     // When button is pressed, g_trigger_cycle returns true and performs the rotation for the servo
//     // If button is not pressed, then it is stopped and does nothing.
//     while (1) {
//         if (g_trigger_cycle) {
//             g_trigger_cycle = false;   // consume event
//             perform_cycle();
//             
//             vTaskDelay(pdMS_TO_TICKS(100)); // small post-cycle delay
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(10));
//         }
//     }
// }

