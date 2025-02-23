#include "Oencoder.h"

void Oencoder::process_data()
{
    uint32_t current_time = HAL_GetTick();
    if (previous_time != 0)
    {
        delta_time = (float)(current_time - previous_time) / 1000.0f;
    }

    left_wheel.current_count = __HAL_TIM_GET_COUNTER(left_wheel.ecoder_tim);
    right_wheel.current_count = __HAL_TIM_GET_COUNTER(right_wheel.ecoder_tim);

    calc_delta_count(&left_wheel);
    calc_delta_count(&right_wheel);

    robot_delta_x = (double)left_wheel.delta_count * pos_2_rpm * wheel_p * (-COS45) - (double)right_wheel.delta_count * pos_2_rpm * wheel_p * COS45;
    robot_delta_y = (double)left_wheel.delta_count * pos_2_rpm * wheel_p * COS45 - (double)right_wheel.delta_count * pos_2_rpm * wheel_p * (COS45);

    world_delta_x = robot_delta_x * (double)(arm_cos_f32(IMU->get_yaw_rad()))-robot_delta_y * (double)(arm_sin_f32(IMU->get_yaw_rad()));

    world_delta_y = robot_delta_x * (double)(arm_sin_f32(IMU->get_yaw_rad())) + robot_delta_y * (double)(arm_cos_f32(IMU->get_yaw_rad()));

    world_pos_x += world_delta_x;
    world_pos_y += world_delta_y;

    data_send[0] = (float)world_pos_x;
    data_send[1] = (float)world_pos_y;
    data_send[2] = (float)IMU->get_heading();

    sendFloatData(1, data_send, 3);

    previous_time = current_time;
}
void Oencoder::calc_delta_count(encoderwheel_t *wheel)
{
    if (wheel->last_count < overflow_min && wheel->current_count > overflow_max)
    {
        wheel->delta_count = -(wheel->last_count + max_count - wheel->current_count);
    }
    else if (wheel->last_count > overflow_max && wheel->current_count < overflow_min)
    {
        wheel->delta_count = wheel->current_count + max_count - wheel->last_count;
    }
    else
    {
        wheel->delta_count = wheel->current_count - wheel->last_count;

        wheel->delta_count = -wheel->delta_count;
    }
    wheel->last_count = wheel->current_count;
}
void Oencoder::add_imu(imu *IMU_)
{
    IMU = IMU_;
}

Oencoder::Oencoder(TIM_HandleTypeDef *left_encoder_tim, TIM_HandleTypeDef *right_encoder_tim, float wheel_perimeter_)
{
    left_wheel.ecoder_tim = left_encoder_tim;
    right_wheel.ecoder_tim = right_encoder_tim;
}

void Oencoder::pos_tf()
{
}

void Oencoder::init()
{
    HAL_TIM_Encoder_Start(left_wheel.ecoder_tim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(right_wheel.ecoder_tim, TIM_CHANNEL_ALL);
}