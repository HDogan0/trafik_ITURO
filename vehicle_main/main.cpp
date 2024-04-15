#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "tusb.h"
#include "string.h"

// Pin Definitions
#define LED PICO_DEFAULT_LED_PIN
#define LencA 16
#define LencB 17
#define Ldir1 5
#define Ldir2 4
#define Lpwm 1

#define RencA 2
#define RencB 3
#define Rdir1 6
#define Rdir2 7
#define Rpwm 15

#define stby 14

//  Message
typedef struct{
    float l_speed;
    float r_speed;
}Message;
Message message = { 0 };
uint8_t magicNumber = 31;

//  Globals
volatile uint32_t R_velocity, L_velocity, lcurrTime, lprevTime, rcurrTime, rprevTime;
float targetVel = 100;

uint32_t cTime, pTime, lTime;

typedef struct{
    float rpm, preRpm, filtRpm;
    float err, errIntegral;
    uint sliceNum, dir1, dir2, pwm;
    float Kp, Ki;
}pidProperties;
pidProperties leftMotor = { 0 }, rightMotor = {0};


//  Function Definitions
void pid(pidProperties *pid);

void encoder_callback(uint gpio, uint32_t events);

void power(uint8_t dir, uint8_t duty, uint slice_num, uint dir1, uint dir2, uint pwm);
uint pwm_setup(uint8_t pin);

float f_abs(float var);
void loopTime();

TU_ATTR_WEAK void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char);


//  Critical Section Definition
critical_section_t crit_velocity;

int main(){
    stdio_init_all();
    sleep_ms(100);

    critical_section_init(&crit_velocity);
    
    { //    pin setup
        gpio_init(LED);
        gpio_set_dir(LED, GPIO_OUT);

        gpio_init(Ldir1);
        gpio_init(Ldir2);
        gpio_init(Lpwm);
        gpio_init(LencA);
        gpio_init(LencB);

        gpio_init(Rdir1);
        gpio_init(Rdir2);
        gpio_init(Rpwm);
        gpio_init(RencA);
        gpio_init(RencB);

        gpio_init(stby);

        gpio_set_dir(Ldir1, GPIO_OUT);
        gpio_set_dir(Ldir2, GPIO_OUT);
        gpio_set_dir(Lpwm, GPIO_OUT);
        gpio_set_dir(LencA, GPIO_IN);
        gpio_set_dir(LencB, GPIO_IN);

        gpio_set_dir(Rdir1, GPIO_OUT);
        gpio_set_dir(Rdir2, GPIO_OUT);
        gpio_set_dir(Rpwm, GPIO_OUT);
        gpio_set_dir(RencA, GPIO_IN);
        gpio_set_dir(RencB, GPIO_IN);

        gpio_set_dir(stby, GPIO_OUT);


        gpio_put(stby, 1);

    }

    //  Interrupt Definitions
    gpio_set_irq_enabled_with_callback(LencA, 0x04, 1, encoder_callback);
    gpio_set_irq_enabled_with_callback(RencA, 0x04, 1, encoder_callback);
                                        // 0x04 == GPIO_IRQ_EDGE_RISE

    //  usb connection
    tud_cdc_n_set_wanted_char(0, magicNumber);
    while(!tud_cdc_connected()){sleep_ms(10);}
    
    //  PWM SETUP
    leftMotor.sliceNum = pwm_setup(Lpwm);
    rightMotor.sliceNum = pwm_setup(Rpwm);
    leftMotor.Kp = 2; leftMotor.Ki = 11;
    rightMotor.Kp = 2; rightMotor.Ki = 11;
    power(1, 0, leftMotor.sliceNum, Ldir1, Ldir2, Lpwm);
    power(1, 0, leftMotor.sliceNum, Rdir1, Rdir2, Rpwm);

    while(1){
        loopTime();

        //handling velocity in critical section
        critical_section_enter_blocking(&crit_velocity);
        leftMotor.rpm = (L_velocity * 60 / (16*20));
        rightMotor.rpm = (R_velocity * 60 / (16*20));
        critical_section_exit(&crit_velocity);

        pid(&leftMotor);
        pid(&rightMotor);

        sleep_ms(1);
    }
    return 0;
}

// Encoder Callback
void encoder_callback(uint gpio, uint32_t events){  
    if (gpio == RencA) {
        int8_t increment = (gpio_get(RencB) ? 1 : -1);
        rcurrTime = time_us_32();
        float dTime = ((float)(rcurrTime - rprevTime) / 1e6);
        if(dTime != 0){
            R_velocity = increment / dTime;  
        }
        rprevTime = rcurrTime;
        return;
    }

    else if (gpio == LencA) {
        int8_t increment = (gpio_get(LencB) ? 1 : -1);
        lcurrTime = time_us_32();
        float dTime = ((float)(lcurrTime - lprevTime) / 1e6);
        if (dTime != 0)
        {
         L_velocity = increment / dTime;
        }
        lprevTime = lcurrTime;
        return;
    }
}

TU_ATTR_WEAK void tud_cdc_rx_wanted_cb(uint8_t itf, char wantedChar){
    if(wantedChar == magicNumber){
        char buf[100];
        if(tud_cdc_available() >= sizeof(Message)){
            int counter = tud_cdc_read(buf, sizeof(Message));
            if(counter == sizeof(Message)){
                memcpy(&message, buf, sizeof(Message));
            }
        }
    }
}

//  motor driver
void power(uint8_t dir, uint8_t duty, uint slice_num, uint dir1, uint dir2, uint pwm){
        uint32_t level = (12500 * duty) / 100;
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pwm), level);
    if(dir == 1){ 
        gpio_put(dir1, 1);
        gpio_put(dir2, 0);
    }
    else { 
        gpio_put(dir1, 0);
        gpio_put(dir2, 1);
    }
}
//  setting up pwm pin and getting slice number
uint pwm_setup(uint8_t pin){
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice_num, 12500 - 1);
    pwm_set_enabled(slice_num, true);
    return slice_num;
}

//float absolute value function
float f_abs(float var){
    if (var < 0.0){var *= -1;}
    return var;
}

//  pid tuner
void pid(pidProperties *pid){
    //filtering velocity (25hz cutoff frequency-low pass filter)
    pid->filtRpm = (0.854 * pid->filtRpm) + (0.0728 * pid->rpm) + (0.0728 * pid->preRpm);
    pid->preRpm = pid->rpm;
    
    pid->err = targetVel - pid->filtRpm;
    pid->errIntegral = pid->errIntegral + pid->err*lTime/1e6;
    
    float u = pid->Kp * pid->err + pid->Ki * pid->errIntegral;
    float pwr = (int)f_abs(u);

    if(pwr > 100){pwr = 100;}
    if(u < 0){power(0, pwr, pid->sliceNum, pid->dir1, pid->dir2, pid->pwm);}
    else{power(1, pwr, pid->sliceNum, pid->dir1, pid->dir2, pid->pwm);}
}

void loopTime(){
    pTime = cTime;
    cTime = time_us_32();
    lTime = cTime - pTime;
}
