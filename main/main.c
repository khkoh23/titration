#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "ae_uart.h"
#include "tanmone_uart.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define ROS_NAMESPACE "" //#define ROS_NAMESPACE CONFIG_MICRO_ROS_NAMESPACE

#define RS1_TX (GPIO_NUM_17)
#define RS1_RX (GPIO_NUM_18)
#define RS2_TX (GPIO_NUM_19)		// usb-jtag
#define RS2_RX (GPIO_NUM_20)		// usb-jtag

static const char *AE_UART_TAG = "ae_uart";
static const char *TANMONE_UART_TAG = "tanmone_uart";
static const char *MICRO_ROS_TASK_TAG = "micro_ros";

enum micro_ros_agent_state {
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
} uros_state;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t ph_publisher, temperature_publisher;
rcl_timer_t ph_timer, temperature_timer;
rcl_subscription_t pipettecmd_subscriber;
std_msgs__msg__Float32 ph_msg, temperature_msg;
std_msgs__msg__Int8 pipettecmd_msg;
const int micro_ros_timeout_ms = 100; // Timeout for each micro ros ping attempt
const uint8_t micro_ros_attempts = 1; // Number of micro ros ping attempts
const uint64_t micro_ros_spin_timeout = RCL_MS_TO_NS(1); // Spin period for micro ros executor
uint16_t micro_ros_i; // Counter for micro ros time sync
const int micro_ros_sync_session_timeout_ms = 50; // Timeout for micro ros time sync

uint8_t ae_device_id = 1, new_ae_device_id; // AE pipette device id
uint8_t ae_task = 0; // 1: readDeviceId, 2: writeDeviceId, 3: readSetVolume, 4: writeSetVolume, 5: readPipetteSpeed, 6: writePipetteSpeed, 7: cmdAspire, 8: cmdDispense, 9: cmdDispenseStepVolume, 10: cmdZero
uint16_t ae_set_volume; // unit: 0.1uL
uint16_t ae_pipette_speed; // high byte: aspire speed, low byte: dispense speed
uint16_t ae_step_volume; // <=actual volume high 8bits, low 8bits

static size_t uart_port = UART_NUM_0;


/* -------------------- ae uart -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

const int ae_uart_buffer_size = 128;
const uint8_t ae_uart_read_tout = 3; // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
uart_config_t ae_uart_config = {
	.baud_rate = 9600,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	// .rx_flow_ctrl_thresh = 122,
	.source_clk = UART_SCLK_DEFAULT, 
};

void ae_uart_help_response() {

}


void ae_uart_task(void *arg) {  
	while (1) {
		switch (ae_task) {
			case 1: // readDeviceId
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: readDeviceId");
				uint8_t read_device_id = ae_uart_readDeviceId();
				break;
			case 2: // writeDeviceId
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: writeDeviceId");
				ae_uart_writeDeviceId(ae_device_id, new_ae_device_id);
				break;
			case 3: // readSetVolume
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: readSetVolume");
				uint16_t read_set_volume = ae_uart_readSetVolume(ae_device_id);
				break;
			case 4: // writeSetVolume
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: writeSetVolume");
				ae_uart_writeSetVolume(ae_device_id, ae_set_volume);
				break;
			case 5: // readPipetteSpeed
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: readPipetteSpeed");
				uint16_t read_pipette_speed = ae_uart_readPipetteSpeed(ae_device_id);
				break;
			case 6: // writePipetteSpeed
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: writePipetteSpeed");
				ae_uart_writePipetteSpeed(ae_device_id, ae_pipette_speed);
				break;
			case 7: // cmdAspire
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: cmdAspire");
				ae_uart_cmdAspire(ae_device_id);
				break;
			case 8: // cmdDispense
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: cmdDispense");
				ae_uart_cmdDispense(ae_device_id);
				break;
			case 9: // cmdDispenseStepVolume
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: cmdDispenseStepVolume");
				ae_uart_cmdDispenseStepVolume(ae_device_id, ae_step_volume); 
				break;
			case 10: // cmdZero
				// ESP_LOGI(AE_UART_TAG, "ae_uart_task: cmdZero");
				ae_uart_cmdZero(ae_device_id);
				break;
			default:
				break;
		}
		ae_task = 0;
		ae_uart_help_response();
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	vTaskDelete(NULL);
}


/* -------------------- micro ros -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void ph_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&ph_publisher, &ph_msg, NULL));
	}
}

void temperature_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
	}
}

void pipettecmd_callback(const void * msgin) { // Aspire: -1, Dispense: 1, Zero: 0
	switch (pipettecmd_msg.data) {
		case -1:
			ae_task = 7;
			break;
		case 1:
			ae_task = 8;
			break;
		case 0:
			ae_task = 10;
			break;
		default:
			break;
	}
	// new_speed = true;
}

bool create_entities(void) {
	allocator = rcl_get_default_allocator();
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); // create init_options
	RCCHECK(rclc_node_init_default(&node, "titration_mcu", ROS_NAMESPACE, &support)); // create node
	//RCCHECK(rclc_publisher_init_default(&debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, Debug), "debug"));
	RCCHECK(rclc_publisher_init_default(&ph_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "ph"));
	RCCHECK(rclc_publisher_init_default(&temperature_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "temperature"));
	//RCCHECK(rclc_subscription_init_default(&speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, SpeedCmd), "speed"));
	RCCHECK(rclc_subscription_init_default(&pipettecmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "pipette_cmd"));
	//RCCHECK(rclc_service_init_default(&hardwareconfig_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(shoalbot_interfaces, srv, HardwareConfig), "hardware_config"));
	RCCHECK(rclc_timer_init_default2(&ph_timer, &support, RCL_MS_TO_NS(100), ph_timer_callback, true));
	RCCHECK(rclc_timer_init_default2(&temperature_timer, &support, RCL_MS_TO_NS(100), temperature_timer_callback, true));
	executor = rclc_executor_get_zero_initialized_executor(); 
	RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); // create executor
	RCCHECK(rclc_executor_add_timer(&executor, &ph_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &temperature_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &pipettecmd_subscriber, &pipettecmd_msg, &pipettecmd_callback, ON_NEW_DATA));
	//RCCHECK(rclc_executor_add_service(&executor, &hardwareconfig_service, &hardwareconfig_req, &hardwareconfig_res, hardwareconfigservice_callback));
	return true;
}

void destroy_entities(void) {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	RCCHECK(rcl_publisher_fini(&ph_publisher, &node));
	RCCHECK(rcl_publisher_fini(&temperature_publisher, &node));
	RCCHECK(rcl_timer_fini(&ph_timer));
	RCCHECK(rcl_timer_fini(&temperature_timer));
	//RCCHECK(rcl_subscription_fini(&speed_subscriber, &node));
	//RCCHECK(rcl_service_fini(&hardwareconfig_service, &node));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
}

void micro_ros_task(void * arg) {
    while(1) {
        switch (uros_state) {
            case WAITING_AGENT: // Check for agent connection
                uros_state = (RMW_RET_OK == rmw_uros_ping_agent(micro_ros_timeout_ms, micro_ros_attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
                break;
            case AGENT_AVAILABLE: // Create micro-ROS entities
                uros_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (uros_state == WAITING_AGENT) { // Creation failed, release allocated resources
                    destroy_entities();
                };
                break;
            case AGENT_CONNECTED: // Check connection and spin on success
                uros_state = (RMW_RET_OK == rmw_uros_ping_agent(micro_ros_timeout_ms, micro_ros_attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (uros_state == AGENT_CONNECTED) {
                    RCSOFTCHECK(rclc_executor_spin_some(&executor, micro_ros_spin_timeout));
                }
                break;
            case AGENT_DISCONNECTED: // Connection is lost, destroy entities and go back to first step
                destroy_entities();
                uros_state = WAITING_AGENT;
                break;
            default:
                break;
        }
		if (micro_ros_i > 1000) { // micro ros time sync after 5 s
			micro_ros_i = 0;
			rmw_uros_sync_session(micro_ros_sync_session_timeout_ms);
		}
		else micro_ros_i ++;
		vTaskDelay(pdMS_TO_TICKS(5));
    }
	vTaskDelete(NULL);
}


/* -------------------- app main -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void app_main(void) {
	uros_state = WAITING_AGENT;

	// uint8_t my_message1[] = {0x03, 0x06, 0x05, 0x05};
	// uint16_t my_message1_length = sizeof(my_message1);
	// uint16_t crc_result1 = ae_uart_CRC16_Modbus(my_message1, my_message1_length);
	// ESP_LOGI(AE_UART_TAG, "Checksum   Low: %x   High: %x", (uint8_t) crc_result1&0x00FF, (uint8_t) ((crc_result1&0xFF00)>>8));

	// uint8_t my_message2[] = {0x00, 0x03, 0x02, 0x02, 0xAE};
	// uint16_t my_message2_length = sizeof(my_message2);
	// uint16_t crc_result2 = tanmone_uart_ModRTU_CRC(my_message2, my_message2_length);
	// ESP_LOGI(TANMONE_UART_TAG, "Checksum   Low: %x   High: %x", (uint8_t) crc_result2&0x00FF, (uint8_t) ((crc_result2&0xFF00)>>8));

	ESP_LOGI(AE_UART_TAG,"Install ae uart driver");
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, ae_uart_buffer_size * 2, 0, 0, NULL, 0)); // no tx buffer, no event queue
	ESP_LOGI(AE_UART_TAG,"Configure ae uart parameter");
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &ae_uart_config));
	ESP_LOGI(AE_UART_TAG,"Assign signals of ae uart peripheral to gpio pins");
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, RS2_TX, RS2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	// ESP_LOGI(NC_UART_TAG,"Set nc uart to rs485 half duplex mode");
	// ESP_ERROR_CHECK(uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX));
	// ESP_LOGI(NC_UART_TAG,"Set nc uart read threshold timeout for TOUT feature");
	// ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_2, nc_uart_read_tout));
	ESP_LOGI(AE_UART_TAG, "Discard all data in the ae uart rx buffer");
	ESP_ERROR_CHECK(uart_flush(UART_NUM_2));

	ESP_LOGI(AE_UART_TAG, "Create ae uart task");
	xTaskCreate(ae_uart_task, "ae_uart_task", 4096, NULL, 5, NULL);

	#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(true, (void *) &uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write, esp32_serial_read);
	#else
	#error micro-ROS transports misconfigured
	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

	ESP_LOGI(MICRO_ROS_TASK_TAG, "Create micro ros task");
	vTaskDelay(pdMS_TO_TICKS(1000));
	xTaskCreate(micro_ros_task, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
