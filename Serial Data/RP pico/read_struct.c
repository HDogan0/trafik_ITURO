#include "pico/stdlib.h"
#include "stdio.h"
#include "tusb.h"

//	struct definition
typedef struct{
    int sleep_time;
}Message;
Message message;

const uint led_pin = PICO_DEFAULT_LED_PIN; 

int main() {

    int LED_STATE = 1;
    message.sleep_time = 100;	//default sleep time

    gpio_init(led_pin);			// setting up built_in led
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_put(led_pin, LED_STATE);

    stdio_init_all();	//  initializing stdio
    sleep_ms(1000);
    while (!tud_cdc_connected()) {
        sleep_ms(10);  // Wait for USB connection
    }
    while (1) {
       
       if (tud_cdc_available()) {
            int bytesRead = tud_cdc_read(&message, sizeof(message));
            //	reading struct as bytes
            if (bytesRead == sizeof(message)) {
                printf("%d \n", message.sleep_time);
            }
        }

        for(uint8_t x = 0; x < 5; x++){
            gpio_put(led_pin, LED_STATE);
            sleep_ms(message.sleep_time);
            gpio_put(led_pin, !LED_STATE);
            sleep_ms(message.sleep_time);
        }
        sleep_ms(100);
    }
}

