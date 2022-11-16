#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

enum Direction {
    foward,
    reverse
};

typedef struct
{
    uint8_t id;
    uint16_t divCounter;
    uint slice_num;
} Motor;

void Motor_construct(Motor* const p_this, uint8_t id);
void Motor_motorOn(Motor* const p_this, uint8_t direction, float speed);
void Motor_motorOff(Motor* const p_this);//, uint8_t motor);
