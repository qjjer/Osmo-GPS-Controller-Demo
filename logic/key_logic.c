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

#include <time.h>
#include "key_logic.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"

static const char *TAG = "LOGIC_KEY";

// 按键事件变量，用于存储当前按键事件
// Key event variable used to store current key event
static key_event_t current_key_event = KEY_EVENT_NONE;

// 按键状态，标记当前按键是否被按下
// Key state flag indicating whether the key is currently pressed
static bool key_pressed = false;

// 按键按下的起始时间，用于计算按下持续时间
// Start time when key is pressed, used to calculate press duration
static TickType_t key_press_start_time = 0;

// 长按阈值（例如：按下超过 1 秒认为是长按）
// Long press threshold (e.g., press longer than 1 second is considered long press)
#define LONG_PRESS_THRESHOLD pdMS_TO_TICKS(1000)

// 单击与长按事件检测时间间隔（例如：50ms）
// Time interval for detecting single press and long press events (e.g., 50ms)
#define KEY_SCAN_INTERVAL pdMS_TO_TICKS(50)

/**
 * @brief 处理长按事件
 *        Handle long press event
 * 
 * 当按键被长时间按下时（超过长按阈值），执行相关的逻辑操作：
 * When the key is pressed for a long time (exceeding the threshold), execute the following operations:
 * 1. 初始化数据层。
 * Initialize data layer.
 * 2. 重新连接 BLE。
 * Reconnect BLE.
 * 3. 建立与相机的协议连接。
 * Establish protocol connection with camera.
 * 4. 获取设备版本信息并订阅相机状态。
 * Get device version info and subscribe to camera status.
 */
static void handle_boot_long_press() {
    /* 初始化数据层 */
    /* Initialize data layer */
    if (!is_data_layer_initialized()) {
        ESP_LOGI(TAG, "Data layer not initialized, initializing now...");
        data_init(); 
        data_register_status_update_callback(update_camera_state_handler);
        data_register_new_status_update_callback(update_new_camera_state_handler);
        if (!is_data_layer_initialized()) {
            ESP_LOGE(TAG, "Failed to initialize data layer");
            return;
        }
    }

    /* 重新连接 BLE */
    /* Reconnect BLE */
    connect_state_t current_state = connect_logic_get_state();

    if (current_state >= BLE_INIT_COMPLETE) {
        ESP_LOGI(TAG, "Current state is %d, disconnecting Bluetooth...", current_state);
        int res = connect_logic_ble_disconnect();
        if (res == -1) {
            ESP_LOGE(TAG, "Failed to disconnect Bluetooth.");
            return;
        }
    }

    ESP_LOGI(TAG, "Attempting to connect Bluetooth...");
    int res = connect_logic_ble_connect(false);
    if (res == -1) {
        ESP_LOGE(TAG, "Failed to connect Bluetooth.");
        return;
    } else {
        ESP_LOGI(TAG, "Successfully connected Bluetooth.");
    }

    /* 相机协议连接 */
    /* Camera protocol connection */
    uint32_t g_device_id = 0x12345678;                           // 示例设备ID / Example device ID
    uint8_t g_mac_addr_len = 6;                                  // MAC地址长度 / MAC address length
    int8_t g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC}; // 示例MAC地址 / Example MAC address
    uint32_t g_fw_version = 0x00;                                // 示例固件版本 / Example firmware version
    uint8_t g_verify_mode = 0;                                   // 首次配对 / First pairing
    uint16_t g_verify_data = 0;                                  // 随机校验码 / Random verification code
    uint8_t g_camera_reserved = 0;                               // 相机编号 / Camera number

    srand((unsigned int)time(NULL));
    g_verify_data = (uint16_t)(rand() % 10000);
    res = connect_logic_protocol_connect(
        g_device_id,
        g_mac_addr_len,
        g_mac_addr,
        g_fw_version,
        g_verify_mode,
        g_verify_data,
        g_camera_reserved
    );
    if (res == -1) {
        ESP_LOGE(TAG, "Failed to connect to camera.");
        return;
    } else {
        ESP_LOGI(TAG, "Successfully connected to camera.");
    }

    /* 获取设备版本信息并打印 */
    /* Get and print device version information */
    version_query_response_frame_t *version_response = command_logic_get_version();
    if (version_response != NULL) {
        free(version_response);
    }

    /* 订阅相机状态 */
    /* Subscribe to camera status */
    res = subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
    if (res == -1) {
        ESP_LOGE(TAG, "Failed to subscribe to camera status.");
        return;
    } else {
        ESP_LOGI(TAG, "Successfully subscribed to camera status.");
    }
}
/**
 * @brief 处理单击事件
 *        Handle single press event
 * 
 * 当按键被单击时，执行以下操作：
 * When the key is single pressed, perform the following operations:
 * 1. 获取当前相机模式。
 * Get current camera mode.
 * 2. 如果相机正在直播，则启动录制。
 * If camera is live streaming, start recording.
 * 3. 如果相机正在录制，则停止录制。
 * If camera is recording, stop recording.
 */
static void handle_boot_single_press() {
    // 获取当前相机模式
    // Get current camera mode
    camera_status_t current_status = current_camera_status;
    camera_mode_t current_mode = current_camera_mode;

    // 处理不同的相机状态，这里也可以用 按键上报（0011）方式实现拍录控制
    // Handle different camera states, recording control can also be implemented using Key Reporting (0011) method
    if (current_mode == CAMERA_MODE_PHOTO || current_status == CAMERA_STATUS_LIVE_STREAMING) {
        // 如果当前模式是拍照、直播，开始录制
        // If current mode is photo or live streaming, start recording
        ESP_LOGI(TAG, "Camera is live streaming. Starting recording...");
        record_control_response_frame_t *start_record_response = command_logic_start_record();
        if (start_record_response != NULL) {
            ESP_LOGI(TAG, "Recording started successfully.");
            free(start_record_response);
        } else {
            ESP_LOGE(TAG, "Failed to start recording.");
            // 尝试唤醒
            // Try to wake up
            connect_logic_ble_wakeup();
        }
    } else if (is_camera_recording()) {
        // 如果当前模式是拍照或录制中，停止录制
        // If current mode is photo or recording, stop recording
        ESP_LOGI(TAG, "Camera is recording or pre-recording. Stopping recording...");
        record_control_response_frame_t *stop_record_response = command_logic_stop_record();
        if (stop_record_response != NULL) {
            ESP_LOGI(TAG, "Recording stopped successfully.");
            free(stop_record_response);
        } else {
            ESP_LOGE(TAG, "Failed to stop recording.");
        }
    } else {
        ESP_LOGI(TAG, "Camera is in an unsupported mode for recording.");
    }

    /* QS 快速切换模式（可放入其他按键） */
    /* QS quick switch mode (can be assigned to other keys) */
    // key_report_response_frame_t *key_report_response = command_logic_key_report_qs();
    // if (key_report_response != NULL) {
    //     free(key_report_response);
    // }

    /* Switch camera to Hyperlapse mode */
    /* 切换相机至运动延时模式 */
     camera_mode_switch_response_frame_t *switch_response = command_logic_switch_camera_mode(CAMERA_MODE_HYPERLAPSE);
     if (switch_response != NULL) {
         free(switch_response);
     }

    /* 发送原始字节测试 */
    /* Send raw bytes test */
    // command_logic_send_raw_bytes("AA1600010000000000128C23001103010000263B43CC", 5000);

    /* 休眠状态下快照键上报测试 */
    /* Snapshot key report test */
    // connect_logic_ble_wakeup();
    // key_report_response_frame_t *snapshot_response = command_logic_key_report_snapshot();
    // if (snapshot_response != NULL) {
    //     free(snapshot_response);
    // }
}

/**
 * @brief 按键扫描任务
 *        Key scan task
 * 
 * 定期检查按键状态，检测单击和长按事件，并触发相应的操作：
 * Periodically check key status, detect single press and long press events, and trigger corresponding operations:
 * - 长按：进行蓝牙断开、重连、相机协议连接等操作。
 * - Long press: perform Bluetooth disconnect, reconnect, camera protocol connection, etc.
 * - 单击：根据当前相机模式启动或停止录制。
 * - Single press: start or stop recording based on current camera mode, and switch camera mode.
 */
static void key_scan_task(void *arg) {
    while (1) {
        // 获取按键状态
        // Get key state
        bool new_key_state = gpio_get_level(BOOT_KEY_GPIO);

        if (new_key_state == 0 && !key_pressed) { // 按键按下 / Key pressed
            key_pressed = true;
            key_press_start_time = xTaskGetTickCount();
            current_key_event = KEY_EVENT_NONE;
            // ESP_LOGI(TAG, "BOOT key pressed.");
        } else if (new_key_state == 0 && key_pressed) { // 按键保持按下状态 / Key remains pressed
            TickType_t press_duration = xTaskGetTickCount() - key_press_start_time;

            if (press_duration >= LONG_PRESS_THRESHOLD && current_key_event != KEY_EVENT_LONG_PRESS) {
                // 长按事件（持续按下达到阈值时立即触发）
                // Long press event (triggered immediately when threshold is reached)
                current_key_event = KEY_EVENT_LONG_PRESS;
                // 处理长按事件：首先断开当前蓝牙连接，然后尝试重新连接
                // Handle long press event: first disconnect current Bluetooth connection, then try to reconnect
                handle_boot_long_press();
                // ESP_LOGI(TAG, "Long press detected. Duration: %lu ticks", press_duration);
            }
        } else if (new_key_state == 1 && key_pressed) { // 按键松开 / Key released
            key_pressed = false;
            TickType_t press_duration = xTaskGetTickCount() - key_press_start_time;

            if (press_duration < LONG_PRESS_THRESHOLD) {
                // 单击事件 / Single press event
                current_key_event = KEY_EVENT_SINGLE;
                ESP_LOGI(TAG, "Single press detected. Duration: %lu ticks", press_duration);
                // 处理单击事件：拍录控制 / Handle single press event: recording control
                handle_boot_single_press();
            }

            // 可以不做额外操作，因为长按的触发已经在按下过程中处理了
            // No additional operation needed as long press is handled during the press
        }

        vTaskDelay(KEY_SCAN_INTERVAL);  // 每隔一段时间扫描一次按键状态 / Scan key state periodically
    }
}

/**
 * @brief 初始化按键逻辑
 *        Initialize key logic
 * 
 * 配置按键的 GPIO 引脚，并启动按键扫描任务。
 * Configure GPIO pin for key and start key scan task.
 */
void key_logic_init(void) {
    // 配置引脚为输入
    // Configure pin as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_KEY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);

    // 启动按键扫描任务
    // Start key scan task
    xTaskCreate(key_scan_task, "key_scan_task", 2048, NULL, 2, NULL);
}

/**
 * @brief 获取当前按键事件
 *        Get current key event
 * 
 * 获取并重置当前的按键事件，主要用于外部任务获取事件后进行处理。
 * Get and reset current key event, mainly used for external tasks to process after getting the event.
 * 
 * @return key_event_t 当前按键事件类型 / Current key event type
 */
key_event_t key_logic_get_event(void) {
    key_event_t event = current_key_event;
    current_key_event = KEY_EVENT_NONE; // 获取后重置事件 / Reset event after getting it
    return event;
}
