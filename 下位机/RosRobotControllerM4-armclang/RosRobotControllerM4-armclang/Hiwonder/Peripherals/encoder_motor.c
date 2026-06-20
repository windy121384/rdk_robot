/**
 * @file encoder_motor.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 编码器电机硬件无关代码
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "encoder_motor.h"
#include "gpio.h"

/**
 * @brief 编码器电机速度测量更新
 * @detials
 * @param self 编码器电机对象指针
 * @param period 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
 * @param counter 编码器当前计数值
 * @retval None.
*/
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t counter)
{
    int delta_count = (int)(self->counter & 0xFFFF) - (int)counter;
    if(delta_count > 30000) delta_count -= 60000;
    if(delta_count < -30000) delta_count += 60000;
    self->counter = counter;
    self->tps = (float)delta_count / period * 0.9f + self->tps * 0.1f;
    self->rps = self->tps / self->ticks_per_circle;
}

/**
 * @brief 编码器电机速度控制任务
 * @detials 编码器电机速度PID控制任务,需要定时指定以完成PID控制更新
 * @param self 编码器电机对象指针
 * @param period 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
 * @retval None.
*/
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period)
{
    float pulse = 0;
    if(!HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin)) {
        pid_controller_update(&self->pid_controller, self->rps, period);
        pulse = self->pid_controller.output;
        pulse = pulse > 1000 ?  1000 : pulse;
        pulse = pulse < -1000 ? -1000 : pulse;
    }
    self->set_pulse(self, pulse > -50 && pulse < 50 ? 0 : pulse);
    self->current_pulse = pulse;
}


/**
 * @brief 编码器电机设置PID控制目标速度
 * @param self self 编码器电机对象指针
 * @param rps 目标速度， 单位转每秒
 * @retval None.
 */
void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps)
{
    rps = rps > self->rps_limit ? self->rps_limit : (rps < -self->rps_limit ? -self->rps_limit : rps); /* 对速度进行限幅 */
    self->pid_controller.set_point = rps; /* 设置 PID 控制器目标 */
}


/**
 * @breif 编码器电机对象初始化
 * @param self 编码器电机对象指针
 * @retval None.
*/
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self)
{
    self->counter = 0;
    self->overflow_num = 0;
    self->tps = 0;
    self->rps = 0;
    self->current_pulse = 0;
    self->ticks_overflow = 0;
    self->ticks_per_circle = 9999; /* 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写 */
    pid_controller_init(&self->pid_controller, 0, 0, 0);
}

