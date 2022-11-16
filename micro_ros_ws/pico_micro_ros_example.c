#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "motor_driver.h"
#include "quadrature_encoder.pio.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

float limit_func(float x, float x_min, float x_max);
float pid_control_v(float input_v, float target_v, float delta_time);
float pid_control_w(float input_w, float target_w, float delta_time);

rcl_publisher_t enc_r_publisher;
std_msgs__msg__Int32 enc_r_msg;

rcl_publisher_t enc_l_publisher;
std_msgs__msg__Int32 enc_l_msg;

rcl_subscription_t joy_twist_subscriber;
geometry_msgs__msg__Twist joy_twist_msg;

rcl_subscription_t det_twist_subscriber;
geometry_msgs__msg__Twist det_twist_msg;

float left_wheel;
float right_wheel;
Motor motor_r, motor_l;

const float wheel_radius = 0.035; //[m] 70mm/2
const float base_width = 0.141; //[m] 149mm - 8mm
const float ticks_per_revolution = 1440.0;
const float Ke = 0.1412429;
const float Vbatt = 3.3;

PIO pio_r = pio0;
PIO pio_l = pio1;
const uint sm_r = 0;
const uint sm_l = 0;
int32_t prev_ticks_r = 0;
int32_t prev_ticks_l = 0;

float x = 0.0;
float y = 0.0;
float theta = 0.0;
float x_end = 0.0;
float y_end = 0.0;
float theta_end = 0.0;

// PID
#define Kp 2.0f
#define Ki 1.5f
#define Kd 0.0f

float joy_v, joy_w;
float det_v, det_w;
float target_v, target_w;
float prev_error_v = 0.0, integ_error_v = 0.0;
float prev_error_w = 0.0, integ_error_w = 0.0;

float w_s;

// timer cb
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    float delta_time = (float)last_call_time / 1.0e9;
    //printf("Timer: time since last call %f\n", (float) last_call_time / 1.0e9);

    // Set Target Velocity
    target_v = joy_v + det_v;
    target_w = joy_w + det_w;
    /*
    if ( (det_v == 0.0f) && (det_w == 0.0f) )
    {
        target_v = joy_v;
        target_w = joy_w;
    }
    else {
        target_v = det_v;
        target_w = det_w;
    }*/

    // Get Encoder Values
    int32_t ticks_r = quadrature_encoder_get_count(pio_r, sm_r);
    int32_t ticks_l = -quadrature_encoder_get_count(pio_l, sm_l);

    enc_r_msg.data = ticks_r;
    enc_l_msg.data = ticks_l;
    //printf("position_r %8d, position_l %8d\n", enc_r_msg.data, enc_l_msg.data);

    RCSOFTCHECK(rcl_publish(&enc_r_publisher, &enc_r_msg, NULL));
    RCSOFTCHECK(rcl_publish(&enc_l_publisher, &enc_l_msg, NULL));

    int32_t delta_ticks_r = ticks_r - prev_ticks_r;
    int32_t delta_ticks_l = ticks_l - prev_ticks_l;

    prev_ticks_r = ticks_r;
    prev_ticks_l = ticks_l;
    //printf("d_tick_r:%d, d_tick_l:%d, dt:%f\n", delta_ticks_r, delta_ticks_l, delta_time);

    float delta_r = (2 * M_PI * wheel_radius * delta_ticks_r) / ticks_per_revolution;
    float delta_l = (2 * M_PI * wheel_radius * delta_ticks_l) / ticks_per_revolution;

    /*
    float w_re = delta_r / delta_time;
    float w_le = delta_l / delta_time;

    float v_e = (w_re + w_le) * wheel_radius / 2.0f;
    float w_e = (w_re - w_le) * wheel_radius / base_width;
    */
    float delta_center = (delta_r + delta_l) * wheel_radius / 2.0f;
    float phi = (delta_r - delta_l) * wheel_radius / base_width;

    float v_e = delta_center / delta_time;
    float w_e = phi / delta_time;

    //odom_update(delta_center, phi, v_e, w_e);

    float pid_v = pid_control_v(v_e, target_v, delta_time);
    float pid_w = pid_control_w(w_e, target_w, delta_time);
    //float pid_w = pid_control_w(w_s, target_w, delta_time);

    float w_r, w_l;
    w_r = ((target_v + pid_v) + (target_w + pid_w) * base_width / 2.0) / wheel_radius; // right motor [rad/s]
    w_l = ((target_v + pid_v) - (target_w + pid_w) * base_width / 2.0) / wheel_radius; // left motor [rad/s]

    if ( (fabs(w_r) < 1.0e-5f) && (fabs(w_l) < 1.0e-5f) )
    {
        //motorDriver_stop();
        Motor_motorOff(&motor_r);
        Motor_motorOff(&motor_l);
    }
    else
    {
        float e_r = Ke * w_r * 3.3;
        float e_l = Ke * w_l * 3.3;

        float duty_r = limit_func(e_r / Vbatt * 100.0f, -100.0f, 100.0f);
        float duty_l = limit_func(e_l / Vbatt * 100.0f, -100.0f, 100.0f);
        //printf("duty_r:%f, duty_l:%f\n", duty_r, duty_l);

        Motor_motorOn(&motor_r, (duty_r < 0.f), fabs(duty_r));
        Motor_motorOn(&motor_l, (duty_l < 0.f), fabs(duty_l));
    }
}

// twist message cb
void joy_twist_sub_cb(const void *msgin)
{
    // Get Velocity from Joystick
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    joy_v = msg->linear.x / 50.0f; // 0.02 m/s
    joy_w = msg->angular.z / 10.0f; // 0.1rad/s
}

void det_twist_sub_cb(const void *msgin)
{
    // Get Velocity from Object Detection
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    det_v = msg->linear.x / 50.0f; // 0.02 m/s
    det_w = msg->angular.z / 10.0f; // 0.1rad/s
}

float pid_control_v(float input_v, float target_v, float delta_time)
{    
    float error_v = target_v - input_v;
    integ_error_v += (error_v + prev_error_v) / 2.0f * delta_time;
    float deriv_error_v = (error_v - prev_error_v) / delta_time;
    prev_error_v = error_v;

    if ((target_v == 0.0f) && (error_v == 0.0f)) integ_error_v = 0.0;

    float output_v = Kp * error_v + Ki * integ_error_v + Kd * deriv_error_v;

    //printf("tar_v:%f, cur_v:%f, err_v:%f, pid_v:%f\n", target_v, input_v, error_v, output_v);

    //return limit_func(output, -1.0f, 1.0f);
    return output_v;
}

float pid_control_w(float input_w, float target_w, float delta_time)
{    
    float error_w = target_w - input_w;
    integ_error_w += (error_w + prev_error_w) / 2.0f * delta_time;
    float deriv_error_w = (error_w - prev_error_w) / delta_time;
    prev_error_w = error_w;

    if ((target_w == 0.0f) && (error_w == 0.0f)) integ_error_w = 0.0;

    float output_w = Kp * error_w + Ki * integ_error_w + Kd * deriv_error_w;

    //printf("tar_w:%f, cur_w:%f, err_w:%f, pid_w:%f\n", target_w, input_w, error_w, output_w);

    //return limit_func(output, -1.0f, 1.0f);
    return output_w;
}

float limit_func(float x, float x_min, float x_max)
{
    float y;

    if (x > x_max)
    {
        y = x_max;
    }
    else if (x < x_min)
    {
        y = x_min;
    }
    else
    {
        y = x;
    }

    return y;
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    stdio_init_all();

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rclc_executor_t executor;
    rclc_executor_t exe_sense;
    
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    RCCHECK(rcl_init_options_init(&init_options, allocator));

    size_t domain_id = (size_t)22;
    const char * node_name = "micro_ros_pico_node";

    RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
    printf("Initializing RCL '%s' with ROS Domain ID %ld...\n", node_name, domain_id);

    //create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));
    
    // create timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(33),//100), //Odometry Publish Frequency[Hz]
        timer_callback));
    
    // create subscriber
    RCCHECK(rclc_subscription_init_best_effort(
      &joy_twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/joy_cmd_vel"));

    // create subscriber
    RCCHECK(rclc_subscription_init_best_effort(
      &det_twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      //"/det_cmd_vel"));
      "/cmd_vel"));

    RCCHECK(rclc_publisher_init_best_effort(
        &enc_r_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "enc_r_values"));

    RCCHECK(rclc_publisher_init_best_effort(
        &enc_l_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "enc_l_values"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &joy_twist_subscriber, &joy_twist_msg, &joy_twist_sub_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &det_twist_subscriber, &det_twist_msg, &det_twist_sub_cb, ON_NEW_DATA));

    // Motor
    Motor_construct(&motor_r, 0);
    Motor_construct(&motor_l, 1);

    // Encoder
    const uint PIN_AB_r = 0;
    const uint PIN_AB_l = 26;

    // ADC pins(GP26-28) have their digital input disabled by default on startup
    gpio_set_input_enabled(PIN_AB_l, true);
    gpio_set_input_enabled(PIN_AB_l + 1, true);

    uint offset_r = pio_add_program(pio_r, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_r, sm_r, offset_r, PIN_AB_r, 0);

    uint offset_l = pio_add_program(pio_l, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_l, sm_l, offset_l, PIN_AB_l, 0);

    enc_r_msg.data = 0;
    enc_l_msg.data = 0;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        //sleep_ms(10);
    }
    
    // free resources
    RCCHECK(rcl_subscription_fini(&joy_twist_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&det_twist_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&enc_r_publisher, &node));
    RCCHECK(rcl_publisher_fini(&enc_l_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
	
    return 0;
}
