#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "ae_uart.h"
#include "tanmone_uart.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static const char *AE_UART_TAG = "AE_UART";
static const char *TANMONE_UART_TAG = "TANMONE_UART";
static const char *UROS_TASK_TAG = "UROS_TASK";

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
	}
}

void micro_ros_task(void * arg) {
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "titration", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "freertos_int32_publisher"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback, true));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.data = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void) {
	uint8_t my_message1[] = {0x03, 0x06, 0x05, 0x05};
	uint16_t my_message1_length = sizeof(my_message1);
	uint16_t crc_result1 = ae_uart_CRC16_Modbus(my_message1, my_message1_length);
	ESP_LOGI(AE_UART_TAG, "Checksum   Low: %x   High: %x", (uint8_t) crc_result1&0x00FF, (uint8_t) ((crc_result1&0xFF00)>>8));

	uint8_t my_message2[] = {0x00, 0x03, 0x02, 0x02, 0xAE};
	uint16_t my_message2_length = sizeof(my_message2);
	uint16_t crc_result2 = tanmone_uart_ModRTU_CRC(my_message2, my_message2_length);
	ESP_LOGI(TANMONE_UART_TAG, "Checksum   Low: %x   High: %x", (uint8_t) crc_result2&0x00FF, (uint8_t) ((crc_result2&0xFF00)>>8));

	#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(true, (void *) &uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write, esp32_serial_read);
	#else
	#error micro-ROS transports misconfigured
	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM
	
	ESP_LOGI(UROS_TASK_TAG, "Create uros task");
	vTaskDelay(pdMS_TO_TICKS(1000));

	xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
