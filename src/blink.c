
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_1  // Feather ESP32-S2 user LED (red)

// void app_main(void)
// {
//     gpio_reset_pin(LED_PIN);
//     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

//     while (1) {
//         gpio_set_level(LED_PIN, 1);  // ON
//         vTaskDelay(pdMS_TO_TICKS(500));
//         gpio_set_level(LED_PIN, 0);  // OFF
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
