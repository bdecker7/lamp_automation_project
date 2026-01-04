
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

// ====== Configuration ======
#define TAG                 "SERVO"
#define SERVO_GPIO          (GPIO_NUM_1)             // Try GPIO_NUM_18 if 1 gives issues
#define SERVO_LEDC_MODE     (LEDC_LOW_SPEED_MODE)
#define SERVO_LEDC_TIMER    (LEDC_TIMER_0)
#define SERVO_LEDC_CHANNEL  (LEDC_CHANNEL_0)
#define SERVO_PWM_FREQ_HZ   (50)                     // 50 Hz for hobby servos
#define SERVO_PWM_RES       (LEDC_TIMER_14_BIT)      // ESP32-S2 supports up to 14-bit LS mode
#define SERVO_MAX_DUTY      ((1 << 14) - 1)          // 16383

// --- Button config (active-low recommended) ---
#define BUTTON_GPIO             (GPIO_NUM_9)   // CHANGE to your actual button GPIO
#define BUTTON_ACTIVE_LEVEL     (0)            // 0 if button to GND with pull-up, 1 if to VCC with pull-down
#define BUTTON_DEBOUNCE_MS      (250)          // ignore retriggers within 250 ms


// Start with a conservative, safe range to avoid binding.
// Expand once you confirm stable motion (e.g., to 1000..2000, or your servo's spec).
#define SERVO_SAFE_MIN_US   (1000)
#define SERVO_SAFE_MID_US   (1450)
#define SERVO_SAFE_MAX_US   (1800)

// For angle mapping; adjust these once you know your servo endpoints.
#define SERVO_MIN_ANGLE_DEG (0.0f)
#define SERVO_MAX_ANGLE_DEG (180.0f)

// ===== Globals =====
static volatile bool g_trigger_cycle = false;
static volatile int64_t g_last_press_us = 0;

// Current endpoints that map to angles:
static float pulse_min_us = SERVO_SAFE_MIN_US;
static float pulse_mid_us = SERVO_SAFE_MID_US;
static float pulse_max_us = SERVO_SAFE_MAX_US;

// ===== Button ISR & setup =====
static void IRAM_ATTR button_isr(void *arg)
{
    int level = gpio_get_level(BUTTON_GPIO);
    // Trigger only when we detect the active level (press)
    if (level == BUTTON_ACTIVE_LEVEL) {
        int64_t now_us = esp_timer_get_time();
        if (now_us - g_last_press_us > (int64_t)BUTTON_DEBOUNCE_MS * 1000) {
            g_last_press_us = now_us;
            g_trigger_cycle = true;  // tell the main loop to run one cycle
        }
    }
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask   = (1ULL << BUTTON_GPIO),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = (BUTTON_ACTIVE_LEVEL == 0) ? GPIO_PULLUP_ENABLE  : GPIO_PULLUP_DISABLE,
        .pull_down_en   = (BUTTON_ACTIVE_LEVEL == 1) ? GPIO_PULLDOWN_ENABLE: GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_ANYEDGE  // use ANYEDGE, and gate inside ISR on active level
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL));
}

static inline uint32_t pulsewidth_us_to_duty(float pulse_us)
{
    const float period_us = 1000000.0f / SERVO_PWM_FREQ_HZ;  // 50 Hz -> 20000 us
    float duty_f = (pulse_us / period_us) * SERVO_MAX_DUTY;
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

static inline esp_err_t servo_set_angle(float deg)
{
    // Clamp angle
    if (deg < SERVO_MIN_ANGLE_DEG) deg = SERVO_MIN_ANGLE_DEG;
    if (deg > SERVO_MAX_ANGLE_DEG) deg = SERVO_MAX_ANGLE_DEG;

    // Linear map angle -> pulse
    float span_deg = SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG;
    float span_us  = pulse_max_us - pulse_min_us;
    float pulse_us = pulse_min_us + (deg / span_deg) * span_us;

    ESP_LOGI(TAG, "Angle=%.1f deg -> %.0f us", deg, pulse_us);
    return servo_set_pulse_us(pulse_us);
}

void min_to_max(){
    ESP_LOGI(TAG, "Sweep: MIN->MAX");
        for (int us = (int)pulse_min_us; us <= (int)pulse_max_us; us += 10) {
            servo_set_pulse_us((float)us);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
}

void max_to_min(){
     ESP_LOGI(TAG, "Sweep: MAX->MIN");
        for (int us = (int)pulse_max_us; us >= (int)pulse_min_us; us -= 10) {
            servo_set_pulse_us((float)us);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
}

void angle_test(){
            // Angle test: 0 -> 90 -> 180 -> 90
        ESP_LOGI(TAG, "Angle test");
        servo_set_angle(0);
        vTaskDelay(pdMS_TO_TICKS(500));
        servo_set_angle(90);
        vTaskDelay(pdMS_TO_TICKS(500));
        servo_set_angle(180);
        vTaskDelay(pdMS_TO_TICKS(500));
        servo_set_angle(90);
        vTaskDelay(pdMS_TO_TICKS(500));
}

// void app_main(void)
// {
//     // Configure LEDC timer
//     ledc_timer_config_t timer_conf = {
//         .speed_mode       = SERVO_LEDC_MODE,
//         .timer_num        = SERVO_LEDC_TIMER,
//         .duty_resolution  = SERVO_PWM_RES,
//         .freq_hz          = SERVO_PWM_FREQ_HZ,
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

//     // Configure LEDC channel on your GPIO
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

//     // Move to mid first
//     ESP_ERROR_CHECK(servo_set_pulse_us(pulse_mid_us));
//     vTaskDelay(pdMS_TO_TICKS(500));

 
//     bool flag = true;
//     while (1) {
//         if(){
//             min_to_max();
//             max_to_min();
//         }

//     }
// }
