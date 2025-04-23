#include "config_credentials.h"

#include "uart_um980.h"
#include "ntrip_client.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_http_client.h"

#define TAG "NTRIP_CLIENT"

#define GGA_BUFFER_SIZE 256

typedef struct {
    char data[GGA_BUFFER_SIZE]; 
    size_t length;
} gga_message_t;

static QueueHandle_t gga_queue;
#define GGA_QUEUE_SIZE 1

QueueHandle_t rtcm_queue;

static void gga_forward_task(void *pvParameters);
static void ntrip_client_task(void *pvParameters);
static esp_http_client_handle_t connect_to_ntrip_server(void);
static esp_err_t send_gga_message(esp_http_client_handle_t client, const char *gga_message, size_t gga_len);
static esp_err_t read_rtcm_corrections(esp_http_client_handle_t client, QueueHandle_t rtcm_queue);

void ntrip_client_init(void)
{
    xTaskCreate(gga_forward_task,
                "gga_forward_task",
                4096,
                NULL,
                7,
                NULL);

    xTaskCreate(ntrip_client_task,
                "ntrip_client_task",
                4096,
                NULL,
                8,
                NULL);
}

static void gga_forward_task(void *pvParameters)
{
    uint8_t *gga_buffer = malloc(GGA_BUFFER_SIZE);
    if (gga_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for GGA location report");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        memset(gga_buffer, 0, GGA_BUFFER_SIZE);

        size_t gga_message_len = 0;

        if (xQueueReceive(uart_um980_gga_to_ntrip_caster_queue,
                          gga_buffer,
                          portMAX_DELAY) == pdTRUE)
        {
            gga_buffer[GGA_BUFFER_SIZE - 1] = '\0';

            char *cr_pos = strchr((char *)gga_buffer, '\r');
            if (cr_pos == NULL)
            {
                continue;
            }

            if (*(cr_pos + 1) == '\n')
            {
                gga_message_len = (cr_pos - (char *)gga_buffer) + 2;
            }
            else
            {
                gga_message_len = (cr_pos - (char *)gga_buffer) + 1;
            }

            gga_buffer[gga_message_len] = '\0';

            gga_message_t message;
            memcpy(message.data, gga_buffer, gga_message_len);
            message.length = gga_message_len;
            
            xQueueOverwrite(gga_queue, &message);
        }
    }

    free(gga_buffer);
}

static void ntrip_client_task(void *pvParameters)
{
    esp_http_client_handle_t client = NULL;
    bool connected = false;

    // Create the queues
    gga_queue = xQueueCreate(GGA_QUEUE_SIZE, sizeof(gga_message_t));

    while (1)
    {
        if (!connected)
        {
            ESP_LOGI(TAG, "Connecting to NTRIP server...");
            client = connect_to_ntrip_server();
            if (client != NULL)
            {
                connected = true;
                ESP_LOGI(TAG, "Connected to NTRIP server");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to connect to NTRIP server, retrying in 5 seconds");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }

        gga_message_t message;
        if (xQueueReceive(gga_queue, &message, 10) == pdTRUE)
        {
            if (send_gga_message(client, message.data, message.length) != ESP_OK)
            {
                ESP_LOGE(TAG, "Error sending GGA message");

                connected = false;
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                client = NULL;
                continue;
            }
        }

        if (read_rtcm_corrections(client, rtcm_queue) != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading RTCM corrections");

            connected = false;
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            client = NULL;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Function to establish a connection with the NTRIP server
static esp_http_client_handle_t connect_to_ntrip_server(void)
{
    esp_http_client_config_t config = {
        .host = NTRIP_HOST,
        .port = NTRIP_PORT,
        .path = "/" NTRIP_MOUNTPOINT,
        .auth_type = HTTP_AUTH_TYPE_BASIC,
        .username = NTRIP_USER,
        .password = NTRIP_PASSWORD,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 120000, // Increase timeout
        .skip_cert_common_name_check = true,
        .transport_type = HTTP_TRANSPORT_OVER_TCP, // Explicitly specify TCP
        .buffer_size = 2048,
        .buffer_size_tx = 2048,
    };

    // Initialize the HTTP client with our configuration
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return NULL;
    }

    // Set required headers for NTRIP
    // esp_http_client_set_header(client, "Ntrip-Version", "Ntrip/2.0");
    // esp_http_client_set_header(client, "User-Agent", "NTRIP ESP32 Client/1.0");
    // esp_http_client_set_header(client, "Connection", "keep-alive");
    esp_http_client_set_header(client, "Ntrip-Version", "Ntrip/2.0");
    esp_http_client_set_header(client, "User-Agent", "NTRIP ESP32Client/1.0");
    esp_http_client_set_header(client, "Accept", "*/*");

    // Open the connection
    esp_err_t err = esp_http_client_open(client, 0); // 0 for no body in GET request
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return NULL;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0)
    {
        ESP_LOGE(TAG, "Error fetching HTTP headers: %d", content_length);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    // Check response status
    int status_code = esp_http_client_get_status_code(client);
    if (status_code != 200)
    {
        ESP_LOGE(TAG, "HTTP GET request failed with status code: %d", status_code);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    ESP_LOGI(TAG, "Successfully connected to NTRIP server");
    return client;
}

static esp_err_t send_gga_message(esp_http_client_handle_t client, const char *gga_message, size_t gga_len)
{
    if (client == NULL || gga_message == NULL)
    {
        ESP_LOGE(TAG, "Invalid client handle or GGA message");
        return ESP_FAIL;
    }

    // Prepare POST request with GGA data
    // esp_http_client_set_method(client, HTTP_METHOD_POST);
    // esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

    esp_err_t err = esp_http_client_write(client, gga_message, gga_len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send GGA message: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Sent GGA message: %.*s", gga_len, gga_message);
    return ESP_OK;
}

static esp_err_t read_rtcm_corrections(esp_http_client_handle_t client, QueueHandle_t rtcm_queue)
{
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Invalid client handle");
        return ESP_FAIL;
    }

    char rtcm_buffer[1024];
    int read_len;

    // Read data from the HTTP stream
    read_len = esp_http_client_read(client, rtcm_buffer, sizeof(rtcm_buffer));
    if (read_len <= 0)
    {
        if (read_len == 0)
        {
            ESP_LOGW(TAG, "No data received from NTRIP server");
            return ESP_OK; // No data available, not an error
        }
        else
        {
            ESP_LOGE(TAG, "Error reading data from NTRIP server: %d", read_len);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Received %d bytes of RTCM data", read_len);

    // Create a rtcm_buffer to send to the queue
    uint8_t *rtcm_data = (uint8_t *)malloc(read_len);
    if (rtcm_data == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for RTCM data");
        return ESP_ERR_NO_MEM;
    }

    memcpy(rtcm_data, rtcm_buffer, read_len);

    rtcm_data_t rtcm_item = {
        .data = rtcm_data,
        .length = read_len};

    // Send to queue with timeout
    if (xQueueSend(rtcm_queue, &rtcm_item, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to send RTCM data to queue");
        free(rtcm_data);
        return ESP_FAIL;
    }

    free(rtcm_data);
    return ESP_OK;
}
