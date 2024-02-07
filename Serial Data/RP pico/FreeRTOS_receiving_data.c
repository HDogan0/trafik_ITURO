#include "pico/stdlib.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include "semphr.h"

typedef struct{
    int sleep_time;
} Message;
Message message;

SemaphoreHandle_t xMutex;

void led_task(void* pvParameters);
void msg_task(void* pvParameters);

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
int LED_STATE = 1;

int main() {
    stdio_init_all(); // Initializing stdio
    message.sleep_time = 100; // Default sleep time

    gpio_init(LED_PIN); // Setting up built-in led
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, LED_STATE);

    // Create mutex
    xMutex = xSemaphoreCreateMutex();

    while (!tud_cdc_connected()) {
        sleep_ms(10); // Wait for USB connection
    }

    xTaskCreate(led_task, "LED_TASK", 512, NULL, 1, NULL);
    xTaskCreate(msg_task, "MESSAGE_TASK", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    vTaskDelete(NULL);
}

void msg_task(void* pvParameters) {
    while (1) {
        if (xMutex != NULL) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                if (tud_cdc_available()) {
                    int bytesRead = tud_cdc_read(&message, sizeof(message));
                    // Reading struct as bytes
                    if (bytesRead == sizeof(message)) {
                        printf("%d \n", message.sleep_time);
                    }
                }
                xSemaphoreGive(xMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void led_task(void* pvParameters){
    while(1){
        for(uint8_t x = 0; x < 5; x++) {
            gpio_put(LED_PIN, LED_STATE);
            vTaskDelay(pdMS_TO_TICKS(message.sleep_time));
            gpio_put(LED_PIN, !LED_STATE);
            vTaskDelay(pdMS_TO_TICKS(message.sleep_time));
        }
        vTaskDelay(pdMS_TO_TICKS(3000)); //pdMS_TO_TICKS parses the right value of ticks which makes 3000 ms to vTaskDelay function
    }
}
