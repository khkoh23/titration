#include "ae_uart.h"
/*Function code 0x01
*/
bool ae_uart_readDeviceId (uint8_t *data) {
    uint8_t func_ = 0x01;
    uint8_t msg_[] = {0x00, func_};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[4] = {0x00, func_, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[5];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[3] << 8) | buf_read[4];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != 0x00) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        *data = buf_read[2];
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x02
*/
bool ae_uart_writeDeviceId (const uint8_t address, const uint8_t msg) { 
    uint8_t func_ = 0x02;
    uint8_t msg_[] = {address, func_, msg};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[5] = {address, func_, msg, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[5];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[3] << 8) | buf_read[4];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x03
Data presentation: one decimal place from two bytes
UOM: ul, up to 200.0 ul*/
bool ae_uart_readSetVolume (const uint8_t address, uint16_t *data) {
    uint8_t func_ = 0x03;
    uint8_t msg_[] = {address, func_};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[4] = {address, func_, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[6];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[4] << 8) | buf_read[5];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        *data = (buf_read[2] << 8) | buf_read[3];
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x04
*/
bool ae_uart_writeSetVolume (const uint8_t address, const uint16_t msg) { 
    uint8_t func_ = 0x04;
    uint8_t msg_[] = {address, func_, (uint8_t) ((msg&0xFF00)>>8), (uint8_t) msg&0x00FF};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[6] = {address, func_, (uint8_t) ((msg&0xFF00)>>8), (uint8_t) msg&0x00FF, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[4];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[2] << 8) | buf_read[3];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x05
aspire speed, dispense speed*/
bool ae_uart_readPipetteSpeed (const uint8_t address, uint8_t *data1, uint8_t *data2) {
    uint8_t func_ = 0x05;
    uint8_t msg_[] = {address, func_};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[4] = {address, func_, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[6];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[4] << 8) | buf_read[5];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        *data1 = buf_read[2];
        *data2 = buf_read[3];
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x06
aspire speed, dispense speed*/
bool ae_uart_writePipetteSpeed (const uint8_t address, const uint8_t msg1, const uint8_t msg2) { 
    uint8_t func_ = 0x06;
    uint8_t msg_[] = {address, func_, msg1, msg2};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[6] = {address, func_, msg1, msg2, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[4];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[2] << 8) | buf_read[3];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x07
run up in set volume and speed*/
bool ae_uart_cmdAspire (const uint8_t address) { 
    uint8_t func_ = 0x07;
    uint8_t msg_[] = {address, func_};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[4] = {address, func_, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[4];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[2] << 8) | buf_read[3];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x08
puse set volume in set speed*/
bool ae_uart_cmdDispense (const uint8_t address) { 
    uint8_t func_ = 0x08;
    uint8_t msg_[] = {address, func_};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[4] = {address, func_, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[4];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[2] << 8) | buf_read[3];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x09
step volume <= actual volume*/
bool ae_uart_cmdDispenseStepVolume (const uint8_t address, const uint16_t msg) {
    uint8_t func_ = 0x09;
    uint8_t msg_[] = {address, func_, (uint8_t) ((msg&0xFF00)>>8), (uint8_t) msg&0x00FF};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[6] = {address, func_, (uint8_t) ((msg&0xFF00)>>8), (uint8_t) msg&0x00FF, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[4];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[2] << 8) | buf_read[3];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

/*Function code 0x0A
piston come back zero position*/
bool ae_uart_cmdZero (const uint8_t address) {
    uint8_t func_ = 0x0A;
    uint8_t msg_[] = {address, func_};
	uint16_t crc_ = ae_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[4] = {address, func_, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_2, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(ae_uart_write_timeout)));
    uint8_t buf_read[4];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_2, buf_read, length_read, pdMS_TO_TICKS(ae_uart_read_timeout));
    uint16_t received_crc = (buf_read[2] << 8) | buf_read[3];
    uint16_t expected_crc = ae_uart_CRC16(buf_read, length_read - 2);
    // Address check:
    if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == ae_exception_func_) return false; 
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else {
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_2)); 
}

uint16_t ae_uart_CRC16(uint8_t *_pBuf, uint16_t _usLen) {
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    uint16_t usIndex;
    while (_usLen--) {
        usIndex = ucCRCHi ^ *_pBuf++;
        ucCRCHi = ucCRCLo ^ ae_uart_s_CRCHi[usIndex];
        ucCRCLo = ae_uart_s_CRCLo[usIndex];
    }
    return ((uint16_t) ucCRCHi << 8 | ucCRCLo);
}

