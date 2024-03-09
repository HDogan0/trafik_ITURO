#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "tusb.h"

// Pin Definitions
#define LED PICO_DEFAULT_LED_PIN
#define LencA 2
#define LencB 3
#define Ldir1 5
#define Ldir2 4
#define Lpwm 6
#define RencA 12
#define RencB 13
#define Rdir1 10
#define Rdir2 9
#define Rpwm 11
//  Message
typedef struct{
    float l_speed;
    float r_speed;
}Message;
Message message;
//  Globals
volatile uint32_t R_velocity, L_velocity, lcurrTime, lprevTime, rcurrTime, rprevTime;
uint32_t cTime, pTime, lTime;
//  Required Functions
void encoder_callback(uint gpio, uint32_t events);
float f_abs(float var);
TU_ATTR_WEAK void tud_cdc_rx_cb(uint8_t itf);
//  Critical Section Definition
critical_section_t crit_velocity;

//*****************************************************************
//      MOTOR CLASS
//*****************************************************************
class Motor{
    public:
    Motor(uint Dir1, uint Dir2, uint Pwm){
        dir1 = Dir1; dir2 = Dir2; pwm = Pwm;
        pwm_setup();
    }

    void motor_control(uint32_t lTime){
        //  filtering velocity (25hz cutoff frequency-low pass filter)
        filtRpm = (0.854 * filtRpm) + (0.0728 * rpm) + (0.0728 * preRpm);
        preRpm = rpm;
        printf("%f\n", filtRpm);
        //  error
        float err = targetVel - filtRpm;
        errIntegral = errIntegral + err*lTime/1e6;
        //  Riemann sum for integral error
        float u = Kp*err + Ki*errIntegral;
        float pwr = (int)f_abs(u);
        //  Power
        if(pwr > 100){pwr = 100;}
        if(u < 0){power(pwr, 0);}
        else{power(pwr, 1);}
        printf("Target: %f, Actual: %f, Error: %f, Power: %f\n", targetVel, filtRpm, err, errIntegral);

    }

    void power(uint8_t duty, uint8_t dir){
        float level = (12500 * duty) / 100;
        pwm_set_chan_level(sliceNum, pwm_gpio_to_channel(pwm), level);
        if(dir == 1){gpio_put(dir1, 1);gpio_put(dir2, 0);}
        else{gpio_put(dir1, 0);gpio_put(dir2, 1);}
    }

    void pwm_setup(){
        gpio_set_function(pwm, GPIO_FUNC_PWM);
        sliceNum = pwm_gpio_to_slice_num(pwm);
        pwm_set_wrap(sliceNum, 12500 - 1);
        pwm_set_enabled(sliceNum, true);
    }

    float get_rpm(){
        return filtRpm;
    }

    float get_targetVel(){
        return targetVel;
    }

    void set_rpm(uint32_t velocity){
        rpm = (velocity * 60 / (16*20));
    }

    void set_target(float velocity){
        targetVel = velocity;
    }
    private:
    uint dir1;
    uint dir2;
    uint pwm;
    uint sliceNum;
    float targetVel = 50;
    float rpm, filtRpm, preRpm, errIntegral = 0;
    float Kp=3, Ki=0;
};
//*****************************************************************
//      MOTOR CLASS
//*****************************************************************

Motor *lMotor;
Motor *rMotor;
int main(){
    stdio_init_all();
    sleep_ms(100);

    critical_section_init(&crit_velocity);

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
    
    //  Interrupt Definitions
    gpio_set_irq_enabled_with_callback(LencA, 0x04, 1, encoder_callback);
    gpio_set_irq_enabled_with_callback(RencA, 0x04, 1, encoder_callback);
                                        // 0x04 == GPIO_IRQ_EDGE_RISE
    
    //  usb connection
    while(!tud_cdc_connected()){sleep_ms(10);}
    
    //  motors
    lMotor = new Motor(Ldir1, Ldir2, Lpwm);
    rMotor = new Motor(Rdir1, Rdir2, Rpwm);
    sleep_ms(10);
    while(1){
        // taking loop time in microseconds
        pTime = cTime;
        cTime = time_us_32();   
        lTime = cTime - pTime;

        //handling velocity in critical section
        critical_section_enter_blocking(&crit_velocity);
        lMotor->set_rpm(L_velocity);
        rMotor->set_rpm(R_velocity);
        critical_section_exit(&crit_velocity);

        lMotor->motor_control(lTime);
        rMotor->motor_control(lTime);
        
        // printf("%f ", lMotor->get_targetVel());
        // printf(" %f\n", rMotor->get_targetVel());

        // printf("\n\n");

        // printf("%f ", lMotor->get_rpm());
        // printf(" %f \n", rMotor->get_rpm());

        sleep_ms(1);
    }
    return 0;
}

// Encoder Callback
void encoder_callback(uint gpio, uint32_t events){
    if (gpio == LencA) {
        int8_t increment = (gpio_get(LencB) ? 1 : -1);
        lcurrTime = time_us_32();
        float dTime = ((float)(lcurrTime - lprevTime) / 1e6);
        L_velocity = increment / dTime;
        lprevTime = lcurrTime;
        return;
    }

    else if (gpio == RencA) {
        int8_t increment = (gpio_get(RencB) ? 1 : -1);
        rcurrTime = time_us_32();
        float dTime = ((float)(rcurrTime - rprevTime) / 1e6);
        R_velocity = increment / dTime;  
        rprevTime = rcurrTime;
        return;
    }
}


float f_abs(float var){
    if (var < 0.0){var *= -1;}
    return var;
}

TU_ATTR_WEAK void tud_cdc_rx_cb(uint8_t itf){
    tud_cdc_read(&message, sizeof(message));
    lMotor->set_target(message.l_speed);
    rMotor->set_target(message.r_speed);
}
