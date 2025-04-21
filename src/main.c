#include "config_credentials.h"
#include "config_board.h"

#include "wifi_connect.h"
#include "wifi_monitor.h"
#include "wifi_led.h"
#include "uart_um980.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

// UART
#define UART_NUM UART_NUM_1
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024

static const char *TAG = "NTRIP_CLIENT";
static QueueHandle_t uart_queue;

// Buffer for UART read/write operations
static uint8_t uart_buffer[BUF_SIZE];
// Buffer to store complete NMEA sentences
static uint8_t gga_buffer[BUF_SIZE];
static size_t gga_len = 0;
static bool found_gga = false;

// HTTP client handle
esp_http_client_handle_t http_client = NULL;

// Function prototypes
static void uart_event_task(void *pvParameters);
static void ntrip_client_task(void *pvParameters);
static bool extract_gga_message(const uint8_t *buffer, size_t len);
static void process_rtk_data(const uint8_t *buffer, size_t len);

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_connect_init();
    
    wifi_monitor_init();
    wifi_led_init(IO_WIFI_LED);
    uart_um980_init();

    ESP_ERROR_CHECK(wifi_connect_sta(WIFI_SSID, WIFI_PASS, portMAX_DELAY));

    // // Configure UART parameters
    // uart_config_t uart_config = {
    //     .baud_rate = UART_BAUD_RATE,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };

    // // Install UART driver and set UART pins
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // // Create tasks for UART event processing and NTRIP client
    // xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    // xTaskCreate(ntrip_client_task, "ntrip_client_task", 8192, NULL, 5, NULL);
}

// Task to handle UART events (UM980 communication)
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    
    // Configure the UM980 to output GGA messages at 1Hz
    //const char *config_cmd = "$JATT,COCOM,1\r\n$JASC,GPGGA,1\r\n";
    //uart_write_bytes(UART_NUM, config_cmd, strlen(config_cmd));
    
    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            memset(uart_buffer, 0, BUF_SIZE);
            switch (event.type) {
                case UART_DATA:
                    uart_get_buffered_data_len(UART_NUM, &buffered_size);
                    int len = uart_read_bytes(UART_NUM, uart_buffer, event.size, pdMS_TO_TICKS(100));
                    
                    // Process the received data
                    if (len > 0) {
                        // Check if this is a GGA message from the UM980
                        if (extract_gga_message(uart_buffer, len)) {
                            found_gga = true;
                        }
                        
                        // Check if this is RTK position data
                        process_rtk_data(uart_buffer, len);
                    }
                    break;
                    
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "UART FIFO overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "UART buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                default:
                    break;
            }
        }
    }
}

// Task to handle NTRIP client operations
static void ntrip_client_task(void *pvParameters)
{
    while (1) {
        // Initialize HTTP client configuration
        esp_http_client_config_t config = {
            .host = NTRIP_HOST,
            .port = NTRIP_PORT,
            .path = "/" NTRIP_MOUNTPOINT,
            .auth_type = HTTP_AUTH_TYPE_BASIC,
            .username = NTRIP_USER,
            .password = NTRIP_PASSWORD,
            .method = HTTP_METHOD_GET,
            .timeout_ms = 120000,  // Increase timeout
            .skip_cert_common_name_check = true,
            .transport_type = HTTP_TRANSPORT_OVER_TCP, // Explicitly specify TCP
            .buffer_size = BUF_SIZE * 2,
            .buffer_size_tx = BUF_SIZE * 2,
        };
        
        ESP_LOGI(TAG, "Initializing HTTP client for NTRIP connection");
        http_client = esp_http_client_init(&config);
        if (http_client == NULL) {
            ESP_LOGE(TAG, "Failed to initialize HTTP client");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Add NTRIP specific headers
        esp_http_client_set_header(http_client, "Ntrip-Version", "Ntrip/2.0");
        esp_http_client_set_header(http_client, "User-Agent", "NTRIP ESP32Client/1.0");
        esp_http_client_set_header(http_client, "Accept", "*/*");
        
        // Open connection to the NTRIP caster
        ESP_LOGI(TAG, "Opening connection to NTRIP caster");
        esp_err_t err = esp_http_client_open(http_client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open connection with NTRIP caster: %s", esp_err_to_name(err));
            esp_http_client_cleanup(http_client);
            http_client = NULL;
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Fetch headers to get HTTP status code
        int content_length = esp_http_client_fetch_headers(http_client);
        int status_code = esp_http_client_get_status_code(http_client);
        ESP_LOGI(TAG, "NTRIP connection HTTP status: %d, Content length: %d", status_code, content_length);
        
        // Check if connection was successful
        if (status_code != 200) {
            ESP_LOGE(TAG, "NTRIP server returned error code: %d", status_code);
            esp_http_client_cleanup(http_client);
            http_client = NULL;
            vTaskDelay(pdMS_TO_TICKS(10000)); // Longer delay after error
            continue;
        }
        
        ESP_LOGI(TAG, "Connected to NTRIP caster successfully");
        
        int data_read_len;
        uint8_t correction_data[BUF_SIZE];
        TickType_t last_gga_sent = 0;
        
        // Main NTRIP client loop
        while (1) {
            // If we have a GGA message, or it's time to send a periodic update
            if (found_gga || (xTaskGetTickCount() - last_gga_sent > pdMS_TO_TICKS(10000))) {
                
                if (found_gga) {
                    // Create a completely new clean buffer for sending
                    char clean_gga[BUF_SIZE];
                    size_t clean_len = 0;
                    
                    // Copy character by character, completely skipping null bytes
                    for (size_t i = 0; i < gga_len; i++) {
                        if (gga_buffer[i] != '\0') {
                            clean_gga[clean_len++] = gga_buffer[i];
                        }
                    }
                    clean_gga[clean_len] = '\0';
                    
                    ESP_LOGI(TAG, "Sending clean GGA to NTRIP caster: %s", clean_gga);
                    
                    // Write directly from the clean buffer
                    int written = esp_http_client_write(http_client, clean_gga, clean_len);
                    
                    if (written > 0) {
                        ESP_LOGI(TAG, "Successfully sent %d bytes to NTRIP caster", written);
                        last_gga_sent = xTaskGetTickCount();
                    } else {
                        ESP_LOGE(TAG, "Failed to send GGA to NTRIP caster: %d", written);
                        break;  // Break the inner loop to reconnect
                    }
                    
                    found_gga = false;
                    gga_len = 0;
                } else {
                    // Send a minimal GGA if we haven't received one recently (keepalive)
                    const char *minimal_gga = "$GPGGA,000000.00,0000.00,N,00000.00,E,0,00,0.0,0.0,M,0.0,M,,*66\r\n";
                    ESP_LOGI(TAG, "Sending minimal GGA to keep connection alive");
                    esp_http_client_write(http_client, minimal_gga, strlen(minimal_gga));
                    last_gga_sent = xTaskGetTickCount();
                }
            }
            
            // Read correction data from the NTRIP caster with a short timeout
            data_read_len = esp_http_client_read_response(http_client, (char*)correction_data, BUF_SIZE);
            
            if (data_read_len > 0) {
                ESP_LOGI(TAG, "Received %d bytes of correction data", data_read_len);
                ESP_LOG_BUFFER_HEXDUMP(TAG, correction_data, (data_read_len > 32) ? 32 : data_read_len, ESP_LOG_INFO);
                
                // Forward the correction data to the UM980 module
                uart_write_bytes(UART_NUM, correction_data, data_read_len);
            } else if (data_read_len == 0) {
                // Normal case when no data is available, no need to warn every time
            } else if (data_read_len < 0) {
                // Get the error from the client instead
                esp_err_t err = esp_http_client_get_errno(http_client);
                ESP_LOGE(TAG, "Error reading from NTRIP caster: %s", esp_err_to_name(err));
                break;  // Break the inner loop to reconnect
            }
            
            vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent CPU hogging
        }
        
        // Clean up and prepare for reconnection
        ESP_LOGW(TAG, "NTRIP connection lost, cleaning up and will reconnect");
        esp_http_client_close(http_client);
        esp_http_client_cleanup(http_client);
        http_client = NULL;
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait before reconnecting
    }
}

// Extract GGA message from UART buffer
static bool extract_gga_message(const uint8_t *buffer, size_t len)
{
    // Look for GPGGA or GNGGA message in the buffer
    char *gga_start = NULL;
    
    for (int i = 0; i < len; i++) {
        if (i + 5 < len && 
            buffer[i] == '$' && 
            (
                (buffer[i+1] == 'G' && buffer[i+2] == 'P' && buffer[i+3] == 'G' && buffer[i+4] == 'G' && buffer[i+5] == 'A') ||
                (buffer[i+1] == 'G' && buffer[i+2] == 'N' && buffer[i+3] == 'G' && buffer[i+4] == 'G' && buffer[i+5] == 'A')
            )) {
            gga_start = (char *)&buffer[i];
            break;
        }
    }
    
    if (gga_start == NULL) {
        return false;
    }
    
    // Find the end of the GGA message - include the entire message up to \r\n
    char *gga_end = strstr(gga_start, "\r\n");
    if (gga_end == NULL) {
        // If we can't find \r\n, try just \n
        gga_end = strchr(gga_start, '\n');
        if (gga_end == NULL) {
            // If we still can't find an end, the message might be incomplete
            return false;
        }
        gga_end++; // Include the \n in the message
    } else {
        gga_end += 2; // Include the \r\n in the message
    }
    
    // Calculate the length of the GGA message
    size_t msg_len = gga_end - gga_start;
    if (msg_len > BUF_SIZE - 1) {
        msg_len = BUF_SIZE - 1;
    }
    
    // Create a temporary buffer to handle the cleanup
    char temp_buffer[BUF_SIZE];
    size_t clean_len = 0;
    
    // Copy character by character, filtering out null bytes
    for (size_t i = 0; i < msg_len; i++) {
        if (gga_start[i] != '\0') {
            temp_buffer[clean_len++] = gga_start[i];
        }
    }
    
    // Ensure proper termination
    temp_buffer[clean_len] = '\0';
    
    // Now copy the cleaned message to gga_buffer
    memcpy(gga_buffer, temp_buffer, clean_len + 1);
    gga_len = clean_len;
    
    ESP_LOGI(TAG, "Extracted clean GGA message: %s", gga_buffer);
    
    return true;
}

// Process RTK position data
static void process_rtk_data(const uint8_t *buffer, size_t len)
{
    // Here we look for NMEA messages that contain the RTK-fixed position
    // For UM980, we typically look for GGA messages with RTK fix indication
    
    // Check if this is a GGA message
    if (extract_gga_message(buffer, len)) {
        // Check fix quality (field 6 in GGA message)
        // 4 = RTK fixed, 5 = RTK float
        char *p = strtok((char*)gga_buffer, ",");
        int field = 1;
        
        while (p != NULL && field < 7) {
            if (field == 6) {
                int fix_quality = atoi(p);
                if (fix_quality == 4) {
                    ESP_LOGI(TAG, "RTK fixed position obtained!");
                    
                    // Now extract the latitude, longitude and height
                    // This would depend on your specific application needs
                    // For demonstration, we'll just log that we got an RTK fix
                    
                    // Here you would add your code to use the RTK fixed position
                    // for your specific application
                    
                } else if (fix_quality == 5) {
                    ESP_LOGI(TAG, "RTK float position obtained");
                } else {
                    ESP_LOGI(TAG, "No RTK fix (quality: %d)", fix_quality);
                }
                break;
            }
            p = strtok(NULL, ",");
            field++;
        }
    }
}