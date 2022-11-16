#include "motor_driver.h"

#define Motor1ForwardPin 2
#define Motor1ReversePin 3
#define Motor2ForwardPin 6
#define Motor2ReversePin 7

static void motor_clear(Motor* const p_this, uint8_t id);

static void motor_clear(Motor* const p_this, uint8_t id)
{
    uint foward_pin, reverse_pin;

    p_this->id = id;

    if (id == 0)
    {
        foward_pin = Motor1ForwardPin;
        reverse_pin = Motor1ReversePin;
    }
    else if (id == 1)
    {
        foward_pin = Motor2ForwardPin;
        reverse_pin = Motor2ReversePin;
    }

    // Allocate GPIO Pins to the PWM
    gpio_set_function(foward_pin, GPIO_FUNC_PWM);
    gpio_set_function(reverse_pin, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to foward_pin
    p_this->slice_num = pwm_gpio_to_slice_num(foward_pin);

    // Set period of 20ms(50Hz)
    // 125*10^6/((warp+1)*clkdiv) = 125*10^6/((24999+1)*100) = 50
    //p_this->divCounter = 25000; // max:65536
    //pwm_set_wrap(p_this->slice_num, p_this->divCounter - 1);
    //pwm_set_clkdiv(p_this->slice_num, 100.0);

    // Set period of 5ms(200Hz)
    // 125*10^6/((warp+1)*clkdiv) = 125*10^6/((12499+1)*50) = 200
    p_this->divCounter = 12500; // max:65536
    pwm_set_wrap(p_this->slice_num, p_this->divCounter - 1);
    pwm_set_clkdiv(p_this->slice_num, 50.0);

    // Set period of 0.1ms(10kHz)
    // NG
    // 125*10^6/((warp+1)*clkdiv) = 125*10^6/((12499+1)*1) = 10000
    //p_this->divCounter = 12500; // max:65536
    //pwm_set_wrap(p_this->slice_num, p_this->divCounter - 1);
    ////pwm_set_clkdiv(p_this->slice_num, 1.0);

    // Set period of 1ms(1kHz)
    // 125*10^6/((warp+1)*clkdiv) = 125*10^6/((12499+1)*10) = 1000
    //p_this->divCounter = 12500; // max:65536
    //pwm_set_wrap(p_this->slice_num, p_this->divCounter - 1);
    //uint8_t div = 10;
    //uint8_t fract = 0;
    //pwm_set_clkdiv_int_frac(p_this->slice_num, div, fract); // 1kHz
}

void Motor_construct(Motor* const p_this, uint8_t id)
{
    motor_clear(p_this, id);
}

void Motor_motorOn(Motor* const p_this, uint8_t direction, float duty)
{
    uint16_t PWM;
    uint16_t pwm_a, pwm_b;

    // cap duty to 0-100%
    if (duty < 0.)
    {
        duty = 0.;
    }
    else if (duty > 100.)
    {
        duty = 100.;
    }
    // convert 0-100 to 0-65535
    PWM = (uint16_t)(p_this->divCounter * duty/100.);

    if (p_this->id == 0)
    {
        if (direction == 0)
        {
            pwm_a = PWM;
            pwm_b = 0;
        }
        else if (direction == 1)
        {
            pwm_a = 0;
            pwm_b = PWM;
        }
        else
        {
            printf("INVALID DIRECTION\n");
        }
    }
    else if (p_this->id == 1)
    {
        if (direction == 0)
        {
            pwm_a = 0;
            pwm_b = PWM;
        }
        else if (direction == 1)
        {
            pwm_a = PWM;
            pwm_b = 0;
        }
        else
        {
            printf("INVALID DIRECTION\n");
        }
    }
    else
    {
        printf("INVALID MOTOR_ID\n");
    }

    // Set channel Duty
    pwm_set_chan_level(p_this->slice_num, PWM_CHAN_A, pwm_a);
    pwm_set_chan_level(p_this->slice_num, PWM_CHAN_B, pwm_b);
    
    // Set the PWM running
    pwm_set_enabled(p_this->slice_num, true);
}

void Motor_motorOff(Motor* const p_this)
{
    Motor_motorOn(p_this, 0, 0.);
}

