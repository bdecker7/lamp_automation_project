
// --- BLE includes (NimBLE Host in ESP-IDF) ---
#include "nimble/ble.h"
#include "host/ble_hs.h"                 // ble_hs_cfg is declared here
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// BLUETOOTH INCLUDES
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"

#include "servo_ble.h"

//for testing purposes
extern void ble_test_run(void);

// If you prefer to trigger the servo cycle from the main loop,
// import the flag here (declare it in your servo file or a shared header).
// extern volatile bool g_trigger_cycle;

// Forward-declare any internal functions used before their definitions.
// This prevents “undeclared” errors due to ordering.
static void nimble_host_task(void *param);
static void ble_on_sync(void);
static int  gap_event_cb(struct ble_gap_event *event, void *arg);
static void ble_advertise_start(void);

// Forward-declare your servo function from your file (if you still call it here):
void small_motor_move(void);


extern volatile bool g_trigger_cycle;

static int servo_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        if (OS_MBUF_PKTLEN(ctxt->om) < 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        uint8_t cmd = ctxt->om->om_data[0];
        // Remember last value if you want to read it later:
        // g_cmd_value = cmd;

        // set a flag and let app_main handle the action
        if (cmd == 0x01) {
            g_trigger_cycle = true;  // your app_main() sees this and runs small_motor_move()
        }

        return 0;
    }
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Return g_cmd_value if you need readback (optional)
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}


// ===== Custom UUIDs (128-bit) =====
static const ble_uuid128_t SERVO_SERVICE_UUID =
    BLE_UUID128_INIT(0x4b,0x91,0x31,0xc3,0xc9,0xc5,0xcc,0x8f,
                     0x9e,0x45,0xb5,0x1f,0x01,0xc2,0xaf,0x4f); // 4fafc201-1fb5-459e-8fcc-c5c9c331914b

static const ble_uuid128_t SERVO_CHAR_UUID =
    BLE_UUID128_INIT(0xa8,0x26,0x1b,0x36,0x07,0xea,0xf5,0xb7,
                     0x88,0x46,0xe1,0x36,0x3e,0x48,0xb5,0xbe); // beb5483e-36e1-4688-b7f5-ea07361b26a8



// ===== GATT database =====
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SERVO_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &SERVO_CHAR_UUID.u,
                .access_cb = servo_char_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 } // end
        },
    },
    { 0 } // end of services
};

// ===== GAP event handler (advertise on disconnect/complete) =====
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            struct ble_gap_adv_params advp = {0};
            advp.conn_mode = BLE_GAP_CONN_MODE_UND;
            advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &advp, gap_event_cb, NULL);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
    case BLE_GAP_EVENT_ADV_COMPLETE: {
        struct ble_gap_adv_params advp = {0};
        advp.conn_mode = BLE_GAP_CONN_MODE_UND;
        advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &advp, gap_event_cb, NULL);
        break;
    }

    default:
        break;
    }
    return 0;
}

// ===== Advertising with our service UUID =====
static void ble_advertise_start(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    const char *name = "ESP32-Servo";
    ble_svc_gap_device_name_set(name);
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.num_uuids128 = 1;
    fields.uuids128 = (ble_uuid128_t *)&SERVO_SERVICE_UUID;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        return;
    }

    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &advp, gap_event_cb, NULL);
    if (rc != 0) {
        // handle error/log if you want
    }
}

// ===== BLE Host sync callback =====
static void ble_on_sync(void)
{
    uint8_t addr_type;
    ble_hs_id_infer_auto(0, &addr_type);
    ble_advertise_start();
}

// ===== Host task (required by nimble_port_freertos_init) =====
static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ===== Public: start BLE (init sequence) =====
void servo_ble_start(void)
{
        // 1) NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2) NimBLE port init (replaces esp_nimble_hci_init in ESP-IDF 5.x)
    nimble_port_init();

    // 3) Register GAP/GATT and our custom services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));

    // 4) Configure host callbacks
    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.sync_cb  = ble_on_sync;

    // 5) Start host thread
    nimble_port_freertos_init(nimble_host_task);
}
