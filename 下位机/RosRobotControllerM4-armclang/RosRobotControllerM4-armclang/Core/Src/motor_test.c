/* motor_test.c
 * Motor PWM test - direct control without PID
 */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "tim.h"
#include "encoder_motor.h"

extern EncoderMotorObjectTypeDef *motors[4];

void motor_pid_test(void)
{
    __HAL_TIM_DISABLE(&htim7);
    
    for(int motor_id = 0; motor_id < 4; motor_id++) {
        if(motors[motor_id] == NULL) continue;
        motors[motor_id]->set_pulse(motors[motor_id], 200);
    }
    for(;;) {
        osDelay(1000);
    }
}
