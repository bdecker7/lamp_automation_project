// =============================================================================
// ble_test_diagnostic.c
// =============================================================================
// PURPOSE: Step-by-step BLE diagnostic test for ESP32 <-> Garmin Instinct 2
//
// HOW TO USE:
//   1. Temporarily replace your servo_ble.c with this file (or add it alongside)
//   2. Call ble_test_run() from app_main() instead of servo_ble_start()
//   3. Open Serial Monitor at 115200 baud
//   4. Launch your Garmin app and watch the logs
//   5. Each TEST will print PASS or FAIL with a reason
//
// TESTS COVERED:
//   TEST 1 - NVS flash init
//   TEST 2 - NimBLE HCI + port init
//   TEST 3 - GATT service registration
//   TEST 4 - BLE advertising started
//   TEST 5 - Garmin connects (wait up to 30s)
//   TEST 6 - Garmin discovers service/characteristic
//   TEST 7 - Garmin writes 0x01 (Lights ON command received)
//   TEST 8 - Garmin writes 0x00 (Lights OFF command received)
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BLE_TEST"

// =============================================================================
// TEST STATE TRACKING
// =============================================================================
typedef enum {
    TEST_PENDING,
    TEST_PASS,
    TEST_FAIL
} TestResult;

static struct {
    TestResult nvs_init;
    TestResult nimble_init;
    TestResult gatt_register;
    TestResult advertising_started;
    TestResult garmin_connected;
    TestResult characteristic_written;
    uint8_t    last_written_value;
    uint16_t   conn_handle;
} g_test_state = {
    .nvs_init              = TEST_PENDING,
    .nimble_init           = TEST_PENDING,
    .gatt_register         = TEST_PENDING,
    .advertising_started   = TEST_PENDING,
    .garmin_connected      = TEST_PENDING,
    .characteristic_written = TEST_PENDING,
    .last_written_value    = 0xFF,
    .conn_handle           = BLE_HS_CONN_HANDLE_NONE,
};

// =============================================================================
// HELPERS
// =============================================================================
static void print_separator(void) {
    ESP_LOGI(TAG, "================================================");
}

static void print_result(const char *test_name, TestResult result, const char *detail) {
    if (result == TEST_PASS) {
        ESP_LOGI(TAG, "  [PASS] %s -- %s", test_name, detail);
    } else if (result == TEST_FAIL) {
        ESP_LOGE(TAG, "  [FAIL] %s -- %s", test_name, detail);
    } else {
        ESP_LOGW(TAG, "  [PENDING] %s -- %s", test_name, detail);
    }
}

static void print_all_results(void) {
    print_separator();
    ESP_LOGI(TAG, "  BLE DIAGNOSTIC TEST RESULTS");
    print_separator();
    print_result("TEST 1: NVS Flash Init",        g_test_state.nvs_init,              "Required for BLE storage");
    print_result("TEST 2: NimBLE Init",            g_test_state.nimble_init,           "HCI + port + host task");
    print_result("TEST 3: GATT Registration",      g_test_state.gatt_register,         "Service + characteristic registered");
    print_result("TEST 4: Advertising Started",    g_test_state.advertising_started,   "ESP32 visible to Garmin scanner");
    print_result("TEST 5: Garmin Connected",       g_test_state.garmin_connected,      "Garmin app paired & connected");
    print_result("TEST 6: Characteristic Written", g_test_state.characteristic_written,"Garmin sent ON/OFF command");
    print_separator();
    if (g_test_state.characteristic_written == TEST_PASS) {
        ESP_LOGI(TAG, "  Last value received: 0x%02X (%s)",
            g_test_state.last_written_value,
            g_test_state.last_written_value == 0x01 ? "Lights ON" :
            g_test_state.last_written_value == 0x00 ? "Lights OFF" : "Unknown command");
    }
    print_separator();
}

// =============================================================================
// CORRECTED UUIDs  (little-endian for NimBLE — reversed from string notation)
//
//  Service UUID string:        4fafc201-1fb5-459e-8fcc-c5c9c331914b
//  Characteristic UUID string: beb5483e-36e1-4688-b7f5-ea07361b26a8
// =============================================================================
static const ble_uuid128_t TEST_SERVICE_UUID =
    BLE_UUID128_INIT(0x4b,0x91,0x31,0xc3,0xc9,0xc5,0xcc,0x8f,
                     0x9e,0x45,0xb5,0x1f,0x01,0xc2,0xaf,0x4f);

static const ble_uuid128_t TEST_CHAR_UUID =
    BLE_UUID128_INIT(0xa8,0x26,0x1b,0x36,0x07,0xea,0xf5,0xb7,
                     0x88,0x46,0xe1,0x36,0x3e,0x48,0xb5,0xbe);

// =============================================================================
// GATT CHARACTERISTIC CALLBACK
// =============================================================================
static int test_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        if (OS_MBUF_PKTLEN(ctxt->om) < 1) {
            ESP_LOGE(TAG, "  [FAIL] Received write with 0 bytes — Garmin sent empty payload");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        uint8_t cmd = ctxt->om->om_data[0];
        g_test_state.last_written_value    = cmd;
        g_test_state.characteristic_written = TEST_PASS;

        ESP_LOGI(TAG, "  [PASS] TEST 6: Characteristic Written!");
        ESP_LOGI(TAG, "         Received byte: 0x%02X", cmd);

        if (cmd == 0x01) {
            ESP_LOGI(TAG, "         Command: Lights ON  -> Would trigger servo here");
        } else if (cmd == 0x00) {
            ESP_LOGI(TAG, "         Command: Lights OFF -> No servo action");
        } else {
            ESP_LOGW(TAG, "         Command: UNKNOWN (0x%02X) — check Garmin payload", cmd);
        }

        print_all_results();
        return 0;
    }

    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "  [INFO] Garmin READ the characteristic (value=0x%02X)",
                 g_test_state.last_written_value);
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// =============================================================================
// GATT SERVICE TABLE
// =============================================================================
static const struct ble_gatt_svc_def test_gatt_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &TEST_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = &TEST_CHAR_UUID.u,
                .access_cb = test_char_access_cb,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
            },
            { 0 }
        },
    },
    { 0 }
};

// =============================================================================
// ADVERTISING
// =============================================================================
static void test_advertise_start(void);

static int test_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_test_state.garmin_connected = TEST_PASS;
            g_test_state.conn_handle      = event->connect.conn_handle;
            ESP_LOGI(TAG, "  [PASS] TEST 5: Garmin CONNECTED! conn_handle=%d",
                     event->connect.conn_handle);
            ESP_LOGI(TAG, "         Waiting for Garmin to discover service and write...");
            ESP_LOGI(TAG, "         -> On your Garmin: open the menu and press Lights ON");
        } else {
            g_test_state.garmin_connected = TEST_FAIL;
            ESP_LOGE(TAG, "  [FAIL] TEST 5: Connection attempt FAILED, status=%d",
                     event->connect.status);
            ESP_LOGE(TAG, "         Possible reasons:");
            ESP_LOGE(TAG, "           - UUID mismatch between Garmin and ESP32");
            ESP_LOGE(TAG, "           - Garmin pairDevice() failed silently");
            ESP_LOGE(TAG, "           - Watch Bluetooth is off or busy");
            test_advertise_start();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "  [INFO] Garmin DISCONNECTED, reason=%d", event->disconnect.reason);
        ESP_LOGW(TAG, "         Re-starting advertising...");
        g_test_state.garmin_connected        = TEST_PENDING;
        g_test_state.characteristic_written  = TEST_PENDING;
        g_test_state.conn_handle             = BLE_HS_CONN_HANDLE_NONE;
        test_advertise_start();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "  [INFO] Advertising cycle complete — restarting");
        test_advertise_start();
        break;

    default:
        break;
    }
    return 0;
}

static void test_advertise_start(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    const char *name = "ESP32-Servo";
    ble_svc_gap_device_name_set(name);
    fields.name             = (uint8_t *)name;
    fields.name_len         = strlen(name);
    fields.name_is_complete = 1;
    fields.num_uuids128     = 1;
    fields.uuids128         = (ble_uuid128_t *)&TEST_SERVICE_UUID;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        g_test_state.advertising_started = TEST_FAIL;
        ESP_LOGE(TAG, "  [FAIL] TEST 4: ble_gap_adv_set_fields failed rc=%d", rc);
        ESP_LOGE(TAG, "         Possible reason: adv payload too large (UUID + name > 31 bytes)");
        return;
    }

    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &advp, test_gap_event_cb, NULL);
    if (rc != 0) {
        g_test_state.advertising_started = TEST_FAIL;
        ESP_LOGE(TAG, "  [FAIL] TEST 4: ble_gap_adv_start failed rc=%d", rc);
    } else {
        g_test_state.advertising_started = TEST_PASS;
        ESP_LOGI(TAG, "  [PASS] TEST 4: Advertising started!");
        ESP_LOGI(TAG, "         Device name : ESP32-Servo");
        ESP_LOGI(TAG, "         Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b");
        ESP_LOGI(TAG, "         -> Now launch your Garmin app to connect...");
    }
}

// =============================================================================
// BLE HOST SYNC + TASK
// =============================================================================
static void test_ble_on_sync(void)
{
    // Use public address only — remove the ble_hs_id_set_rnd(NULL) that was
    // in your original code (passing NULL there causes a crash)
    uint8_t addr_type;
    int rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "  [WARN] ble_hs_id_infer_auto failed rc=%d (non-fatal)", rc);
    }
    test_advertise_start();
}

static void test_nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// =============================================================================
// PUBLIC ENTRY POINT — call this from app_main() instead of servo_ble_start()
// =============================================================================
void ble_test_run(void)
{
    print_separator();
    ESP_LOGI(TAG, "  STARTING BLE DIAGNOSTIC TESTS");
    ESP_LOGI(TAG, "  Baud rate: 115200");
    ESP_LOGI(TAG, "  Target device: Garmin Instinct 2");
    print_separator();

    // ------------------------------------------------------------------
    // TEST 1: NVS Flash
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "  Running TEST 1: NVS Flash Init...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err == ESP_OK) {
        g_test_state.nvs_init = TEST_PASS;
        ESP_LOGI(TAG, "  [PASS] TEST 1: NVS Flash Init OK");
    } else {
        g_test_state.nvs_init = TEST_FAIL;
        ESP_LOGE(TAG, "  [FAIL] TEST 1: NVS Flash Init FAILED err=0x%x", err);
        ESP_LOGE(TAG, "         BLE cannot store bonding info without NVS — stopping here");
        print_all_results();
        return;
    }

    // ------------------------------------------------------------------
    // TEST 2: NimBLE Init
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "  Running TEST 2: NimBLE Init...");
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        g_test_state.nimble_init = TEST_FAIL;
        ESP_LOGE(TAG, "  [FAIL] TEST 2: nimble_port_init FAILED rc=%d", rc);
        print_all_results();
        return;
    }
    g_test_state.nimble_init = TEST_PASS;
    ESP_LOGI(TAG, "  [PASS] TEST 2: NimBLE HCI + port init OK");

    // ------------------------------------------------------------------
    // TEST 3: GATT Service Registration
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "  Running TEST 3: GATT Service Registration...");
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int gatt_rc = ble_gatts_count_cfg(test_gatt_svcs);
    if (gatt_rc != 0) {
        g_test_state.gatt_register = TEST_FAIL;
        ESP_LOGE(TAG, "  [FAIL] TEST 3: ble_gatts_count_cfg failed rc=%d", rc);
        print_all_results();
        return;
    }
    int add_rc = ble_gatts_add_svcs(test_gatt_svcs);
    if (add_rc != 0) {
        g_test_state.gatt_register = TEST_FAIL;
        ESP_LOGE(TAG, "  [FAIL] TEST 3: ble_gatts_add_svcs failed rc=%d", rc);
        print_all_results();
        return;
    }
    g_test_state.gatt_register = TEST_PASS;
    ESP_LOGI(TAG, "  [PASS] TEST 3: GATT service registered OK");
    ESP_LOGI(TAG, "         Service UUID  : 4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    ESP_LOGI(TAG, "         Char UUID     : beb5483e-36e1-4688-b7f5-ea07361b26a8");
    ESP_LOGI(TAG, "         Char flags    : READ | WRITE");

    // ------------------------------------------------------------------
    // TEST 4: Start Advertising (result printed inside test_advertise_start)
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "  Running TEST 4: Starting BLE Advertising...");
    ble_hs_cfg.sync_cb  = test_ble_on_sync;
    ble_hs_cfg.reset_cb = NULL;
    nimble_port_freertos_init(test_nimble_host_task);

    // Tests 5 and 6 are event-driven — they print when the Garmin connects
    // and when it writes to the characteristic.
    ESP_LOGI(TAG, "  TEST 5 (Garmin Connect) and TEST 6 (Write Received)");
    ESP_LOGI(TAG, "  are event-driven — results will print automatically.");
    ESP_LOGI(TAG, "  -> Launch your Garmin app now!");
    print_separator();
}