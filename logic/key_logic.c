/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2025 SZ DJI Technology Co., Ltd.
 *  
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 */

#include "key_logic.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"

static const char *TAG = "LOGIC_KEY";

#define LONG_PRESS_THRESHOLD  pdMS_TO_TICKS(1000)
#define KEY_SCAN_INTERVAL     pdMS_TO_TICKS(50)

typedef struct {
    gpio_num_t    gpio;
    bool          pressed;
    TickType_t    press_start_time;
    key_event_t   event;
} key_state_t;

static key_state_t keys[KEY_ID_MAX] = {
    [KEY_ID_BOOT] = {BOOT_KEY_GPIO, false, 0, KEY_EVENT_NONE},
    [KEY_ID_QS]   = {QS_KEY_GPIO,   false, 0, KEY_EVENT_NONE},
};

// ========== BOOT键处理（不变）==========

static void handle_boot_long_press(void) {
    if (!is_data_layer_initialized()) {
        data_init();
        data_register_status_update_callback(update_camera_state_handler);
        data_register_new_status_update_callback(update_new_camera_state_handler);
    }

    connect_state_t current_state = connect_logic_get_state();
    if (current_state >= BLE_INIT_COMPLETE) {
        connect_logic_ble_disconnect();
    }

    if (connect_logic_ble_connect(false) == -1) return;

    uint32_t g_device_id = 0x12345678;
    uint8_t  g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    uint16_t g_verify_data = (uint16_t)(rand() % 10000);

    connect_logic_protocol_connect(g_device_id, 6, g_mac_addr, 0x00, 0, g_verify_data, 0);

    version_query_response_frame_t *version = command_logic_get_version();
    if (version) free(version);

    subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
}

static void handle_boot_single_press(void) {
    camera_status_t status = current_camera_status;
    camera_mode_t   mode   = current_camera_mode;

    if (mode == CAMERA_MODE_PHOTO || status == CAMERA_STATUS_LIVE_STREAMING) {
        record_control_response_frame_t *resp = command_logic_start_record();
        if (resp) free(resp);
        else connect_logic_ble_wakeup();
    } else if (is_camera_recording()) {
        record_control_response_frame_t *resp = command_logic_stop_record();
        if (resp) free(resp);
    }
}

// ========== QS键处理（只保留快速切换）==========

static void handle_qs_press(void) {
    ESP_LOGI(TAG, "QS: quick switch");
    
    key_report_response_frame_t *resp = command_logic_key_report_qs();
    if (resp) free(resp);
}

// ========== 按键扫描 ==========

static void key_scan_task(void *arg) {
    while (1) {
        for (int i = 0; i < KEY_ID_MAX; i++) {
            bool new_state = (gpio_get_level(keys[i].gpio) == 0);
            
            if (new_state && !keys[i].pressed) {
                keys[i].pressed = true;
                keys[i].press_start_time = xTaskGetTickCount();
                
            } else if (!new_state && keys[i].pressed) {
                keys[i].pressed = false;
                TickType_t duration = xTaskGetTickCount() - keys[i].press_start_time;
                
                if (duration < LONG_PRESS_THRESHOLD) {
                    // 短按处理
                    if (i == KEY_ID_BOOT) {
                        handle_boot_single_press();
                    } else if (i == KEY_ID_QS) {
                        handle_qs_press();  // QS只处理短按
                    }
                } else {
                    // 长按只给BOOT键
                    if (i == KEY_ID_BOOT) {
                        handle_boot_long_press();
                    }
                }
            }
        }
        vTaskDelay(KEY_SCAN_INTERVAL);
    }
}

// ========== 对外接口 ==========

void key_logic_init(void) {
    for (int i = 0; i < KEY_ID_MAX; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << keys[i].gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&io_conf);
    }
    
    ESP_LOGI(TAG, "Key init: BOOT=GPIO%d, QS=GPIO%d", BOOT_KEY_GPIO, QS_KEY_GPIO);
    xTaskCreate(key_scan_task, "key_scan_task", 4096, NULL, 2, NULL);
}

key_event_t key_logic_get_event(void) {
    key_event_t event = keys[KEY_ID_BOOT].event;
    keys[KEY_ID_BOOT].event = KEY_EVENT_NONE;
    return event;
}
