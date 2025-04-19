#include "wifi_monitor.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "WIFI_LED"

typedef enum {
    LED_PATTERN_OFF,
    LED_PATTERN_ON,
    LED_PATTERN_SLOW_BLINK,  // 1Hz
    LED_PATTERN_FAST_BLINK,  // 4Hz
} led_pattern_t;

static int led_gpio = -1;
static led_pattern_t current_pattern = LED_PATTERN_OFF;
static esp_timer_handle_t blink_timer = NULL;
static bool led_state = false;

static void wifi_led_set_pattern(led_pattern_t pattern);
static void wifi_led_set_pattern_task(void *pvParameters);

// Timer callback for LED blinking
static void led_blink_timer_callback(void *arg)
{
    led_state = !led_state;
    gpio_set_level(led_gpio, led_state);
}

// Stop any active blink timer
static void stop_blink_timer(void)
{
    if (blink_timer != NULL) {
        esp_timer_stop(blink_timer);
    }
}

// Start the blink timer with appropriate period
static void start_blink_timer(uint64_t period_us)
{
    stop_blink_timer();
    
    esp_timer_start_periodic(blink_timer, period_us);
}

esp_err_t wifi_led_init(int gpio_pin)
{
    if (gpio_pin < 0) {
        ESP_LOGE(TAG, "Invalid GPIO pin number");
        return ESP_ERR_INVALID_ARG;
    }
    
    led_gpio = gpio_pin;
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << led_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Create timer for blinking
    if (blink_timer == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = led_blink_timer_callback,
            .name = "led_blink_timer"
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_timer));
    }
    
    wifi_led_set_pattern(LED_PATTERN_OFF);
    ESP_LOGI(TAG, "Wifi LED initialized on GPIO %d", led_gpio);
    
    xTaskCreate(&wifi_led_set_pattern_task, 
        "wifi_led_set_pattern_task", 
        2048,
        NULL,
        3,
        &wifi_monitor_led_handler
    );

    return ESP_OK;
}

void wifi_led_set_pattern(led_pattern_t pattern)
{
    if (pattern == current_pattern) {
        return;
    }
    
    current_pattern = pattern;
    
    switch (pattern) {
        case LED_PATTERN_OFF:
            stop_blink_timer();
            gpio_set_level(led_gpio, 0);
            break;
            
        case LED_PATTERN_ON:
            stop_blink_timer();
            gpio_set_level(led_gpio, 1);
            break;
            
        case LED_PATTERN_SLOW_BLINK:
            led_state = true;
            gpio_set_level(led_gpio, led_state);
            start_blink_timer(500 * 1000);
            break;
            
        case LED_PATTERN_FAST_BLINK:
            led_state = true;
            gpio_set_level(led_gpio, led_state);
            start_blink_timer(125 * 1000);
            break;
    }
}

static void wifi_led_set_pattern_task(void *pvParameters)
{
    uint32_t state;
    while (1)
    {
        xTaskNotifyWait(0, 0, &state, portMAX_DELAY);
        
        switch (state)
        {
            case WIFI_MONITOR_CONNECTING:
                wifi_led_set_pattern(LED_PATTERN_SLOW_BLINK);
                break;
            case WIFI_MONITOR_CONNECTED:
                wifi_led_set_pattern(LED_PATTERN_ON);
                break;
            case WIFI_MONITOR_DISCONNECTED:
                wifi_led_set_pattern(LED_PATTERN_OFF);
                break;
            case WIFI_MONITOR_RECONNECTING:
                wifi_led_set_pattern(LED_PATTERN_FAST_BLINK);
                break;
        }
    }
}    