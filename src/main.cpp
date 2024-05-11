/**
 *@file main.cpp
 * @author Alper Tunga Güven (alpertunga2003@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <cstdio>
#include <utility>
#include "pico/stdlib.h"
#include "hardware/pio.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/color_rgba.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include "pico_uart_transports.h"

#include "bno055.h"
}

#include "encoder_substep.hpp"
#include "motor.hpp"
#include "ws2812.hpp"


static rcl_subscription_t rover_movement_subscriber {};
static geometry_msgs__msg__Twist rover_speed_received {};

static rcl_subscription_t led_strip_subscriber {};
static std_msgs__msg__ColorRGBA led_strip_msg_received {};

static rcl_publisher_t motor_publishers[4] {};
static rcl_publisher_t imu_publisher {};

static TaskHandle_t micro_ros_task_handle {};
static TaskHandle_t motor_task_handle[4] {};
static TaskHandle_t bno055_task_handle {};
static TaskHandle_t bme280_task_handle {};
static TaskHandle_t led_strip_task_handle {};

static QueueHandle_t motor_rpm_queues[4] {};
static QueueHandle_t bno055_data_queue {};
static QueueHandle_t led_strip_queue {};

static alarm_pool_t* alarm_pool {};
static bool alarm_pool_set { false };

WS2812 led_strip_g(LED_STRIP_PIN, LED_STRIP_SIZE, pio1, 0, WS2812::FORMAT_GRB);

struct MotorData {
	int32_t target_rpm {};
	int32_t current_rpm {};
	float pwm {};
};

enum MotorPos {
	front_left = 0,
	front_right = 1,
	back_left = 2,
	back_right = 3,
};

#define CLAMP(x, upper, lower) (MIN((upper), MAX((x), (lower))))

constexpr float Kp { static_cast<float>(PWM_MAX_DUTY_CYCLE) / MOTOR_MAX_RPM };
constexpr float Ki { Kp / 10.0f };
// constexpr float Kd { Kp / 50.0f };
constexpr float integral_boundary { PWM_MAX_DUTY_CYCLE };

template <uint pwmPinL, uint pwmPinR, uint encA, MotorPos pos>
bool motor_pid_cb(repeating_timer* t) {
	// All 
	static Motor motor(pwmPinL, pwmPinR);
	static EncoderSubstep encoder(pio0, pos, encA);
	static float integral { 0 };
	static int32_t prev_error { 0 };
	static int64_t alarm_count { 0 };
	// Create the microROS messeage to be sent as current pwm.
	static std_msgs__msg__Float32 motor_msg {
		.data = 0
	};

	int32_t target_rpm_received { *((int32_t*) t->user_data) };

	const int32_t target_rpm = target_rpm_received * 1000;

	const int32_t current_rpm = encoder.getRpm();
	const float error { static_cast<float>(target_rpm - current_rpm) / 1000.0f };

	const float proportional = Kp * error;

	integral = CLAMP(integral + error * Ki, integral_boundary, -integral_boundary);

	// const int32_t deltaT = static_cast<int32_t>(t->delay_us);
	// const float derivative = Kd * (error - prev_error) / (deltaT);
	// prev_error = error;

	const float motor_pwm = CLAMP(proportional + integral, PWM_MAX_DUTY_CYCLE, -PWM_MAX_DUTY_CYCLE);
	motor.setSpeedPercent(motor_pwm);

	if ((alarm_count % 100) == 0) {
		// Publish the current velocity as microROS messeage.
		motor_msg.data = target_rpm_received;
		const auto ret = rcl_publish(&motor_publishers[pos], &motor_msg, NULL);
	}
	alarm_count++;
	return true;
}

void movement_subscription_callback(const void* msgin) {
	// Receive the cmd_vel messeage from the subscription and send it to the freeRTOS queue.
	const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*) msgin;

	const int32_t motor_left_rpm = CLAMP(static_cast<int32_t>(msg->linear.x - msg->angular.z), MOTOR_MAX_RPM, -MOTOR_MAX_RPM);
	const int32_t motor_right_rpm = CLAMP(static_cast<int32_t>(msg->linear.x + msg->angular.z), MOTOR_MAX_RPM, -MOTOR_MAX_RPM);

	gpio_xor_mask(0x01 << PICO_DEFAULT_LED_PIN);

	xQueueOverwrite(motor_rpm_queues[front_left], &motor_left_rpm);
	xQueueOverwrite(motor_rpm_queues[back_left], &motor_left_rpm);

	xQueueOverwrite(motor_rpm_queues[front_right], &motor_right_rpm);
	xQueueOverwrite(motor_rpm_queues[back_right], &motor_right_rpm);
}

void led_strip_subscription_callback(const void* msgin) {

	const std_msgs__msg__ColorRGBA* msg = (const std_msgs__msg__ColorRGBA*) msgin;

	uint32_t rgb = WS2812::RGB(
		static_cast<uint8_t>(CLAMP(msg->r, 255, 0)),
		static_cast<uint8_t>(CLAMP(msg->g, 255, 0)),
		static_cast<uint8_t>(CLAMP(msg->b, 255, 0))
	);

	xQueueOverwrite(led_strip_queue, &rgb);
}

void led_strip_task(void* param) {

	constexpr int led_size = LED_STRIP_SIZE;

	WS2812 led_strip(std::move(led_strip_g));

	uint32_t rgb = WS2812::RGB(0, 255, 0);

	while (true) {
		xQueueReceive(led_strip_queue, &rgb, portMAX_DELAY);

		taskENTER_CRITICAL();
		led_strip.fill(rgb);
		led_strip.show();
		taskEXIT_CRITICAL();
	}
}

void bno055_task(void* param) {

	constexpr uint bno055_connection_retry_time { 1000 };

	taskENTER_CRITICAL();
	rcl_publisher_t bno055_publisher = imu_publisher;
	taskEXIT_CRITICAL();

	// Try to connect to the bno055 sensor and in case of failure wait for the desired cooldown time and retry until success.
	bno055_t bno;
	while (bno055_pico_init(&bno, i2c_default, BNO055_I2C_ADDR1)) {
		vTaskDelay(bno055_connection_retry_time / portTICK_PERIOD_MS);
	}
	sleep_ms(50);

	while (bno055_set_power_mode(BNO055_POWER_MODE_NORMAL)) {
		vTaskDelay(bno055_connection_retry_time / portTICK_PERIOD_MS);

	}
	sleep_ms(50);

	// Set the operation mode to NDOF to get absolute orientation data and automatic runtime calibration.
	while (bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF)) {
		vTaskDelay(bno055_connection_retry_time / portTICK_PERIOD_MS);
	}
	sleep_ms(50);

	auto currentTickCount = xTaskGetTickCount();
	while (true) {
		// Read the calibration status. If the status is 0 or 1, it means the magnetometer picked up some interference and the measurements should not be trusted.
		// If the status is 2 or 3, then the calibration status is good and measurement can be used 
		uint8_t calibrationStatus;
		bno055_get_sys_calib_stat(&calibrationStatus);

		// bno055_accel_float_t accelData {};
		// bno055_convert_float_accel_xyz_msq(&accelData);

		// bno055_gyro_float_t gyroData {};
		// bno055_convert_float_gyro_xyz_dps(&gyroData);

		// Read the data in euler angles and send it to both freeRTOS queue and microROS publisher.
		bno055_euler_float_t eulerAngles {};
		bno055_convert_float_euler_hpr_deg(&eulerAngles);

		geometry_msgs__msg__Vector3 angle_msg {
			.x = eulerAngles.p,
			.y = eulerAngles.r,
			.z = eulerAngles.h
		};
		xQueueOverwrite(bno055_data_queue, &eulerAngles);

		auto ret = rcl_publish(&bno055_publisher, &angle_msg, NULL);

		// Delay the task 10ms so that the data is read at 100Hz which is the maximum speed of bno055.
		xTaskDelayUntil(&currentTickCount, 10 / portTICK_PERIOD_MS);
	}
}

void bme280_task(void* param) {
	vTaskDelete(NULL);
}

void i2c_setup() {

	// Configure the i2c pins.
	gpio_init(I2C_SDA_PIN);
	gpio_init(I2C_SDA_PIN);
	gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_SDA_PIN);
	gpio_pull_up(I2C_SDA_PIN);
	i2c_init(i2c_default, 400 * 1000);

	// Create the i2c device tasks. They all need to run on the same core, 
	// otherwise a mutex needs to be set to prevent 2 cores from accessing the same I2C line at the same time.
	xTaskCreate(bno055_task, "bno055_task", configMINIMAL_STACK_SIZE * 4,
		NULL, configMAX_PRIORITIES - 2, &bno055_task_handle);
	vTaskCoreAffinitySet(bno055_task_handle, ON_CORE_ZERO);

	xTaskCreate(bme280_task, "bme280_task", configMINIMAL_STACK_SIZE * 4,
		NULL, configMAX_PRIORITIES - 3, &bme280_task_handle);
	vTaskCoreAffinitySet(bme280_task_handle, ON_CORE_ZERO);

}

// #define MOTOR_PID_CONTROL

template<uint pwmPinL, uint pwmPinR, uint encA, MotorPos pos>
void motor_task(void* param) {

	constexpr int timeout_for_turnoff_ms { 2000 };

	constexpr uint32_t red = WS2812::RGB(255, 0, 0);
	constexpr uint32_t green = WS2812::RGB(0, 255, 0);

	if (alarm_pool_set == false) {
		alarm_pool = alarm_pool_create_with_unused_hardware_alarm(4);
		alarm_pool_set = true;
	}

	bool motor_is_working = false;

	int32_t target_rpm { 0 };

	repeating_timer timer {};
	alarm_pool_add_repeating_timer_us(alarm_pool, -1000, motor_pid_cb<pwmPinL, pwmPinR, encA, pos>, &target_rpm, &timer);

	while (true) {
		int32_t rpm_received { 0 };
		// Get the linear and angular velocity from the queue.
		if (xQueueReceive(motor_rpm_queues[pos], &rpm_received, pdMS_TO_TICKS(timeout_for_turnoff_ms)) == pdFALSE) {
			target_rpm = 0;
			xQueueOverwrite(led_strip_queue, &red);
			motor_is_working = false;
		} else {
			target_rpm = rpm_received;
			if (motor_is_working != true) {
				motor_is_working = true;
				xQueueOverwrite(led_strip_queue, &green);
			}
		}

	}
}

void micro_ros_task(void* param) {
	// Necessary boilerplate for microROS node creation.
	rcl_allocator_t allocator = rcl_get_default_allocator();

	rclc_support_t support;
	rclc_support_init(&support, 0, NULL, &allocator);

	rcl_node_t node;
	rclc_node_init_default(&node, "pico_rover_node", "", &support);

	// Create 4 motor speed publishers that will send their current speed as their messeage.
	const char* publisher_names[4] { "motor_front_left_msg", "motor_front_right_msg", "motor_back_left_msg", "motor_back_right_msg" };
	for (int i = 0; i < 4; i++) {
		rclc_publisher_init_best_effort(
			&(motor_publishers[i]),
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
			publisher_names[i]
		);
	}
	// Create bno055 publisher that will send its absolute orientation data in euler angles.
	rclc_publisher_init_best_effort(
		&imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
		"bno055_orientation_msg"
	);

	// Create the subscriber that will get data from cmd_vel topic.
	rclc_subscription_init_default(
		&rover_movement_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"
	);

	// Create the subscriber that will get rgb data for led strip.
	rclc_subscription_init_default(
		&led_strip_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
		"led_strip_rgba"
	);

	// rcl_timer_t timer;
	// constexpr unsigned int timer_timeout = 1000;
	// rclc_timer_init_default(
	// 	&timer,
	// 	&support,
	// 	RCL_MS_TO_NS(timer_timeout),
	// 	timer_callback
	// );

	// Create the executor struct that will handle the actual microROS execution.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 7, &allocator);
	// rclc_executor_add_timer(&executor, &timer);
	rclc_executor_add_subscription(&executor, &rover_movement_subscriber, &rover_speed_received, movement_subscription_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &led_strip_subscriber, &led_strip_msg_received, led_strip_subscription_callback, ON_NEW_DATA);

	// Create the freeRTOS queue that will deliver the subcription messeage to the motor tasks in a thread safe way. 
	bno055_data_queue = xQueueCreate(1, sizeof(bno055_euler_float_t));
	led_strip_queue = xQueueCreate(1, sizeof(uint32_t));
	for (int i = 0; i < 4; i++) {
		motor_rpm_queues[i] = xQueueCreate(1, sizeof(int32_t));
	}

	// Suspends the freeRTOS scheduler so that the all newly created tasks can start at the same time.

	vTaskSuspendAll();

	i2c_setup();

	pio_add_program(pio0, &quadrature_encoder_substep_program);

	xTaskCreate(motor_task<FRONT_LEFT_LPWM, FRONT_LEFT_RPWM, FRONT_LEFT_ENCODER_A, front_left>,
		"motor_front_left_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 2, &motor_task_handle[0]);
	vTaskCoreAffinitySet(motor_task_handle[0], ON_CORE_ONE);

	xTaskCreate(motor_task<FRONT_RIGHT_LPWM, FRONT_RIGHT_RPWM, FRONT_RIGHT_ENCODER_A, front_right>,
		"motor_front_right_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 2, &motor_task_handle[1]);
	vTaskCoreAffinitySet(motor_task_handle[1], ON_CORE_ONE);

	xTaskCreate(motor_task<BACK_LEFT_LPWM, BACK_RIGHT_RPWM, BACK_LEFT_ENCODER_A, back_left>,
		"motor_back_left_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 2, &motor_task_handle[2]);
	vTaskCoreAffinitySet(motor_task_handle[2], ON_CORE_ONE);

	xTaskCreate(motor_task<BACK_RIGHT_LPWM, BACK_RIGHT_RPWM, BACK_RIGHT_ENCODER_A, back_right>,
		"motor_back_right_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 2, &motor_task_handle[3]);
	vTaskCoreAffinitySet(motor_task_handle[3], ON_CORE_ONE);

	xTaskCreate(led_strip_task, "led_strip_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 3, &led_strip_task_handle);
	vTaskCoreAffinitySet(led_strip_task_handle, ON_BOTH_CORES);

	xTaskResumeAll();

	// Checks the microRTOS data transfer.
	while (true) {
		rclc_executor_spin(&executor);
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

int main() {

	led_strip_g.fill(WS2812::RGB(255, 0, 0));
	led_strip_g.show();

	// set_sys_clock_khz(270 * 1000, true);
	// sleep_ms(100);

	// Necessary boilerplate code for micro_ros connection.
	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	constexpr const int timeout_ms = 1000;
	constexpr const uint8_t attempts = 120;
	// Try to connect to ROS2 until success.
	while (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK)
	{
		// Unreachable agent, exiting program.
		printf("Could not ping!\n");
	}
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	sleep_ms(100);

	for (int i = 0; i < LED_STRIP_SIZE; i++) {
		led_strip_g.setPixelColor(i, WS2812::RGB(0, 255, 0));
		led_strip_g.show();
		sleep_ms(10);
	}

	// Create the micro_ros task will create all the topics and other tasks.
	xTaskCreate(micro_ros_task, "micro_ros_task", configMINIMAL_STACK_SIZE * 4,
		NULL, configMAX_PRIORITIES - 1, &micro_ros_task_handle);
	vTaskCoreAffinitySet(micro_ros_task_handle, ON_CORE_ZERO);

	// Start the freeRTOS scheduler. The code should never reach the while loop.
	vTaskStartScheduler();

	while (true) {

	}



	return 0;
}

