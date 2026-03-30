#include <esp_err.h>
#include <stdbool.h>

extern volatile bool g_trigger_cycle;
void button_init(void);

void small_motor_move(void);
