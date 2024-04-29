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

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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

static WS2812 ledStrip(
	19,            // Data line is connected to pin 0. (GP0)
	120,         		// Strip is 120 LEDs long.
	pio1,               // Use PIO 0 for creating the state machine.
	0,                  // Index of the state machine that will be created for controlling the LED strip
	// You can have 4 state machines per PIO-Block up to 8 overall.
	// See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
	WS2812::FORMAT_GRB  // Pixel format used by the LED strip
);

static rcl_subscription_t rover_movement_subscriber {};
static geometry_msgs__msg__Twist rover_speed_received {};

static rcl_subscription_t led_strip_subscriber {};
static std_msgs__msg__ColorRGBA led_strip_msg_received {};

static rcl_publisher_t motor_publishers[4] {};
static rcl_publisher_t imu_publisher {};

static TaskHandle_t idle_led_task_handle {};
static TaskHandle_t micro_ros_task_handle {};
static TaskHandle_t motor_task_handle[4] {};
static TaskHandle_t bno055_task_handle {};
static TaskHandle_t bme280_task_handle {};
static TaskHandle_t led_strip_task_handle {};

static QueueHandle_t rover_movement_queue {};
static QueueHandle_t bno055_data_queue {};
// static QueueHandle_t led_strip_queue {};


enum MotorPos {
	front_left = 0,
	front_right = 1,
	back_left = 2,
	back_right = 3,
};
struct RoverVelocity {
	float linear_velocity {};
	float angular_velocity {};
};

#define CLAMP(x, upper, lower) (MIN(upper, MAX(x, lower)))

void movement_subscription_callback(const void* msgin) {
	// Receive the cmd_vel messeage from the subscription and send it to the freeRTOS queue.
	const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*) msgin;

	RoverVelocity rover_vel {
		.linear_velocity = static_cast<float>(msg->linear.x),
		.angular_velocity = static_cast<float>(msg->angular.z)
	};
	BaseType_t xHigherPriorityTaskWoken {};
	constexpr uint32_t led_pin_mask { 0x01 << PICO_DEFAULT_LED_PIN };
	gpio_xor_mask(led_pin_mask);

	BaseType_t taskwoken {};
	xQueueOverwrite(rover_movement_queue, &rover_vel);
	// portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void led_strip_subscription_callback(const void* msgin) {


	const std_msgs__msg__ColorRGBA* msg = (const std_msgs__msg__ColorRGBA*) msgin;

	uint32_t rgb = WS2812::RGB(
		static_cast<uint8_t>(CLAMP(msg->r, 0, 255)),
		static_cast<uint8_t>(CLAMP(msg->g, 0, 255)),
		static_cast<uint8_t>(CLAMP(msg->b, 0, 255))
	);

	ledStrip.fill(rgb);
	ledStrip.show();

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

	constexpr uint i2c_sda { 0 };
	constexpr uint i2c_scl { 1 };

	// Configure the i2c pins.
	gpio_init(i2c_sda);
	gpio_init(i2c_scl);
	gpio_set_function(i2c_sda, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl, GPIO_FUNC_I2C);
	gpio_pull_up(i2c_sda);
	gpio_pull_up(i2c_scl);
	i2c_init(i2c_default, 400 * 1000);

	// Create the i2c device tasks. They all need to run on the same core, 
	// otherwise a mutex needs to be set to prevent 2 cores from accessing the same I2C line at the same time.
	xTaskCreate(bno055_task, "bno055_task", configMINIMAL_STACK_SIZE * 4, NULL, configMAX_PRIORITIES - 2, &bno055_task_handle);
	vTaskCoreAffinitySet(bno055_task_handle, 0x01);

	xTaskCreate(bme280_task, "bme280_task", configMINIMAL_STACK_SIZE * 4, NULL, configMAX_PRIORITIES - 3, &bme280_task_handle);
	vTaskCoreAffinitySet(bme280_task_handle, 0x01);

}

template<uint pwmPinL, uint pwmPinR, uint encA, MotorPos pos>
void motor_task(void* param) {

	// Create the encoder object.
	EncoderSubstep encoder(pio0, (pwmPinL / 2) - 1, encA);
	
	// Create the motor class and set its speed to 0.
	Motor motor(pwmPinL, pwmPinR);
	motor.setSpeedPercent(0.0f);

	RoverVelocity rover_vel {};
	// Create the microROS messeage to be sent as current velocity.
	std_msgs__msg__Float32 motor_msg {
		.data = 0.0f
	};

	auto currentTickCount = xTaskGetTickCount();
	while (true) {
		// Get the linear and angular velocity from the queue.
		xQueuePeek(rover_movement_queue, &rover_vel, 0);

		if ((pos == front_left) || (pos == back_left)) {
			rover_vel.angular_velocity *= -1;
		}

		// Read the current speed from the encoder in encoder counts * 1000 and publish it as messeage.
		const float current_speed { static_cast<float>(encoder.getSpeed()) / 4000.0f };

		// Calculate the desired motor velocity
		float motor_velocity { rover_vel.linear_velocity + rover_vel.angular_velocity };
		if (motor_velocity >= 100.0f)
			motor_velocity = 100.0f;
		else if (motor_velocity <= -100.0f)
			motor_velocity = -100.0f;

		motor.setSpeedPercent(motor_velocity);

		// Publish the current velocity as microROS messeage.
		motor_msg.data = motor_velocity;
		auto ret = rcl_publish(&motor_publishers[pos], &motor_msg, NULL);

		// Delay the task for 10ms.
		xTaskDelayUntil(&currentTickCount, 10 / portTICK_PERIOD_MS);
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
	rover_movement_queue = xQueueCreate(1, sizeof(RoverVelocity));
	// led_strip_queue = xQueueCreate(1, sizeof(uint32_t));
	bno055_data_queue = xQueueCreate(1, sizeof(bno055_euler_float_t));

	// Suspends the freeRTOS scheduler so that the all newly created tasks can start at the same time.

	vTaskSuspendAll();

	i2c_setup();

	pio_add_program(pio0, &quadrature_encoder_substep_program);

	xTaskCreate(motor_task<2, 3, 10, front_left>, "motor_front_left_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 1, &motor_task_handle[0]);
	vTaskCoreAffinitySet(motor_task_handle[0], 0x03);

	xTaskCreate(motor_task<4, 5, 12, front_right>, "motor_front_right_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 1, &motor_task_handle[1]);
	vTaskCoreAffinitySet(motor_task_handle[1], 0x03);

	xTaskCreate(motor_task<6, 7, 14, back_left>, "motor_back_left_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 1, &motor_task_handle[2]);
	vTaskCoreAffinitySet(motor_task_handle[2], 0x03);

	xTaskCreate(motor_task<8, 9, 20, back_right>, "motor_back_right_task", configMINIMAL_STACK_SIZE * 2, nullptr, configMAX_PRIORITIES - 1, &motor_task_handle[3]);
	vTaskCoreAffinitySet(motor_task_handle[3], 0x03);


	ledStrip.fill(WS2812::RGB(0, 255, 0));
	ledStrip.show();

	
	xTaskResumeAll();

	// Checks the microRTOS data transfer.
	while (true) {
		rclc_executor_spin(&executor);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

int main() {

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

	// Create the micro_ros task will create all the topics and other tasks.
	xTaskCreate(micro_ros_task, "micro_ros_task", configMINIMAL_STACK_SIZE * 4, NULL, configMAX_PRIORITIES, &micro_ros_task_handle);
	vTaskCoreAffinitySet(micro_ros_task_handle, 0x03);

	// Start the freeRTOS scheduler. The code should never reach the while loop.
	vTaskStartScheduler();

	while (true) {

	}



	return 0;
}

