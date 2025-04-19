#include "config_board.h"

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "wifi_monitor.h"

#define TAG "UM980"

#define UART_NUM UART_NUM_1
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024
#define MESSAGE_INTERVAL_MS 1000

void um980_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    
    xTaskCreate(
        um980_request_gga_task,
        "um980_request_gga_task",
        4096,
        NULL,
        5,
        &wifi_state_handler
    );
}

void um980_request_gga_task(void *pvParameters)
{
    uint32_t message_count = 0;
    char message_buffer[64];
    uint32_t notification_value;
    
    ESP_LOGI(TAG, "UART message sender task started");
    
    while (1)
    {
        // Wait for initial notification or remain blocked until WiFi connects
        if (message_count == 0) {
            xTaskNotifyWait(0, UINT32_MAX, &notification_value, portMAX_DELAY);
            if (notification_value != WIFI_MONITOR_CONNECTED) {
                // If we get notified but WiFi is not connected, continue waiting
                continue;
            }
        }
        
        // Create and send UART message
        message_count++;
        int len = snprintf(message_buffer, sizeof(message_buffer), 
                          "UART message #%lu - WiFi connected\r\n", message_count);
        uart_write_bytes(UART_NUM, message_buffer, len);
        ESP_LOGI(TAG, "Sent: %s", message_buffer);
        
        // Wait for the next interval or a notification
        if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, pdMS_TO_TICKS(MESSAGE_INTERVAL_MS))) {
            // We received a notification - check if we should keep sending
            if (notification_value != WIFI_MONITOR_CONNECTED) {
                ESP_LOGI(TAG, "WiFi disconnected - pausing UART messages");
                // Clear message count to ensure we wait for connection again
                message_count = 0;
            }
        }
    }
}
