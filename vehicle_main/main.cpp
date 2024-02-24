#include "pico/stdlib.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include "semphr.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

//*****************************************************************
//      PUBLIC VARIABLE DEFINITIONS
//*****************************************************************
uint8_t default_speed = 50;
float angle_x = 0;
//*****************************************************************
//      PUBLIC VARIABLE DEFINITIONS
//*****************************************************************

//*****************************************************************
//      PIN DEFINITIONS
//*****************************************************************
#define lDir1 1
#define lDir2 2
#define lPwm 3

#define rDir1 4
#define rDir2 5
#define rPwm 6

//*****************************************************************
//      PIN DEFINITIONS
//*****************************************************************

//*****************************************************************
//      FreeRTOS DEFINITIONS
//*****************************************************************
TaskHandle_t msg_handler, drv_handler, gy_handler;
void msg_task(void* pvParameters);
void drv_task(void* pvParameters);
void gy_task(void* pvParameters);
SemaphoreHandle_t xMutex;
SemaphoreHandle_t binSemph;
//*****************************************************************
//      FreeRTOS DEFINITIONS
//*****************************************************************

//*****************************************************************
//      OTHER FUNCTION DEFINITIONS
//*****************************************************************
uint abs(float numb);
uint pwm_setup(uint8_t pin);
uint32_t pwm_generator(uint slice_num, uint channel, uint freq, uint duty);
//*****************************************************************
//      OTHER FUNCTION DEFINITIONS
//*****************************************************************

//*****************************************************************
//      TELEMETRY DEFINITIONS
//*****************************************************************
typedef struct{
    int l_motor;
    int r_motor;
    int state;
} Message;
Message message;
//*****************************************************************
//      TELEMETRY DEFINITIONS
//*****************************************************************

//*****************************************************************
//      CLASS DEFINITIONS
//*****************************************************************
class Motor{
    public: 
    Motor(uint8_t pwm_, uint8_t dir1_, uint8_t dir2_){
        pwm = pwm_;
        dir1 = dir1_;
        dir2 = dir2_;
        slice_num = pwm_setup();
        duty_cycle = 0;
    }

    uint pwm_setup(){
        gpio_set_function(pwm, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(pwm);
        pwm_set_wrap(slice_num, 12500 - 1);
        pwm_set_enabled(slice_num, true);
        return slice_num;
    }

    void drive(uint8_t duty, uint8_t dir){
        if(dir == 1){ /*forward*/ }
        else { /*backward*/ }
        duty_cycle = duty;
        uint32_t level = (12500 * duty_cycle) / 100;
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pwm), level);
    }

    private:
    uint8_t pwm;
    uint8_t dir1;
    uint8_t dir2;
    uint slice_num;
    uint8_t duty_cycle;
    
};

class Gyro{
    public:
    #define MPU_ADDR 0x68
    #define PWR_MGMT 0x6B
    #define GYRO_CNFG 0x1B

    float gyro_cal;
    int16_t gyro_raw;
    uint8_t buffer[2], c_buffer[2];

    Gyro(){
        gyro_setup();
    }

    void activate_gyro(uint8_t activate){
        if(activate == 1){vTaskResume(gy_handler);}
        else if(activate == 0){vTaskSuspend(gy_handler);}
    }

    void gyro_setup(){
        i2c_init(i2c_default, 400*1000);
        gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
        gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
        bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

        uint8_t PWR_CNFG[] = {PWR_MGMT, 0x00};
        i2c_write_blocking(i2c_default, MPU_ADDR, PWR_CNFG, 2, false);
    
        uint8_t gyro_config[] = {GYRO_CNFG, 0x10}; // Assuming you want ±1000 degrees/sec
        i2c_write_blocking(i2c_default, MPU_ADDR, gyro_config, sizeof(gyro_config), false);

        gyro_offset();

    }

    void gyro_offset(){
        for(uint16_t x = 0; x < 2000; x++){
            c_buffer[0] = 0x43;
            i2c_write_blocking(i2c_default, MPU_ADDR, c_buffer, 1, true); // true to keep master control of bus
            i2c_read_blocking(i2c_default, MPU_ADDR, c_buffer, 2, false);   // false finished with bus
            gyro_raw = (c_buffer[0] << 8 | c_buffer[1]);
            gyro_cal += gyro_raw;
            sleep_ms(3); 
        }

        gyro_cal /= 2000;
    }
    private:
};

class Vehicle{
    public:
    Vehicle(uint8_t leftPwmPin, uint8_t LD1, uint8_t LD2, uint8_t rightPwmPin, uint8_t RD1, uint8_t RD2): Lmotor(leftPwmPin, LD1, LD2), Rmotor(rightPwmPin, RD1, RD2), gyro(){
        l_duty = 0;
        r_duty = 0;
        xTaskCreate(gy_task, "GY_TASK", 512, (void*)&gyro, 2, &gy_handler);
        xSemaphoreTake(binSemph, portMAX_DELAY);    
        // a binary semaphore should be added because if scheduling starts and this specific task goes on before the argument passes, it may invoke an error.
    }

    void power(){
        Lmotor.drive(l_duty, l_dir);
        Rmotor.drive(r_duty, r_dir);
    }
    
    //  state -> 0
    void stop(){
        l_duty = 0;
        r_duty = 0;
        l_dir = 1;
        r_dir = 1;
        power();
    }
    
    //  state -> 1
    void moveForward(int Rpwm, int Lpwm){
        l_duty = default_speed;
        r_duty = default_speed;
        l_dir = 1;
        r_dir = 1;
        power();
    }

    //  state -> 2
    void moveAroundRight(int16_t degree){
        angle = angle_x;
        gyro.activate_gyro(1);

        gyro.activate_gyro(0);
    }

    // state -> 3
    void moveAroundLeft(){
        angle = angle_x;
        gyro.activate_gyro(1);

        gyro.activate_gyro(0);
    }  

    private:
    Gyro gyro;
    Motor Lmotor;
    Motor Rmotor;
    uint8_t l_duty;
    uint8_t r_duty;
    uint8_t l_dir;
    uint8_t r_dir;
    int state;          //
    int angle;          // angle -> (-180 - 180)
};

//*****************************************************************
//      CLASS DEFINITIONS
//*****************************************************************
int main() {
    stdio_init_all();
    sleep_ms(100);

    xMutex = xSemaphoreCreateMutex();
    while(xMutex == NULL);
    binSemph = xSemaphoreCreateBinary();
    while(binSemph == NULL);
    
    xTaskCreate(msg_task, "MSG_TASK", 512, NULL, 1, &msg_handler);
    xTaskCreate(drv_task, "DRV_TASK", 512, NULL, 1, &drv_handler);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    vTaskStartScheduler();
    
    vTaskDelete(NULL);
    // after creation of tasks, systick gets unnecessary. Deleting systick makes our system more efficient.
}

void msg_task(void* pvParameters){
    while(!tud_cdc_connected()){vTaskDelay(pdMS_TO_TICKS(10));}
    while(1){
        if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE){
            if(tud_cdc_available()){
                int bytesRead = tud_cdc_read(&message, sizeof(Message));
                xSemaphoreGive(xMutex);
            }
        }
    }
}

void drv_task(void* pvParameters){
    Vehicle vehicle(lPwm, lDir1, lDir2, rPwm, rDir1, rDir2);
    while(1){
        //  motor driver
    }
}

void gy_task(void* pvParameters){
    Gyro* gyro = (Gyro*) pvParameters;
    xSemaphoreGive(binSemph);
    float elapsed_time;
    uint32_t previous_time = time_us_32(), current_time;
    float gyro_x; 
    while(1){
        gyro->c_buffer[0] = 0x43;
        i2c_write_blocking(i2c_default, MPU_ADDR, gyro->c_buffer, 1, true); // true to keep master control of bus
        i2c_read_blocking(i2c_default, MPU_ADDR, gyro->c_buffer, 2, false);   // false finished with bus
        gyro->gyro_raw = ((gyro->c_buffer[0] << 8 | gyro->c_buffer[1]));
        gyro->gyro_raw -= gyro->gyro_cal;

        gyro_x = (gyro->gyro_raw) / 32.8;

        current_time = time_us_32();
        elapsed_time = (current_time - previous_time) / 1000000.0;
        previous_time = current_time;

        angle_x += gyro_x * elapsed_time;   
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
