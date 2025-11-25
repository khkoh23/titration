#include "tanmone_uart.h"

/*Function code 0x03, Starting address 0x0000, Quantity of registers: 0x0001
Data presentation: two decimal places from unsigned short 0 to 1400
Range: 0.00 to 14.00 pH*/
bool tanmone_uart_readpH (const uint8_t address, uint16_t *data) { 
    uint8_t func_ = 0x03;
    uint8_t msg_[] = {address, func_, 0x00, 0x00, 0x00, 0x01};
	uint16_t crc_ = tanmone_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[8] = {address, func_, 0x00, 0x00, 0x00, 0x01, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_1, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(tanmone_uart_write_timeout)));
    uint8_t buf_read[7];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_1, buf_read, length_read, pdMS_TO_TICKS(tanmone_uart_read_timeout));
    uint16_t received_crc = (buf_read[5] << 8) | buf_read[6];
    uint16_t expected_crc = tanmone_uart_CRC16(buf_read, length_read - 2);
    // Minimum response length check:
    if (length_read < 5) return false; // response length shorter than address (1) + functioncode (1) + bytecount (1) + crc (2)
    // Address check:
    else if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == (func_ | tanmone_exception_func_)) return false; // 0x83
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else { 
        *data = (buf_read[3] << 8) | buf_read[4];
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_1)); 
} 

/*Function code 0x03, Starting address 0x0001, Quantity of registers 0x0001
Data presentation: one decimal place from short -100 to 1300
Range: -10.0 to 130.0 degC*/
bool tanmone_uart_readTemperature (const uint8_t address, int16_t *data) {  
    uint8_t func_ = 0x03;
    uint8_t msg_[] = {address, func_, 0x00, 0x01, 0x00, 0x01};
	uint16_t crc_ = tanmone_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[8] = {address, func_, 0x00, 0x01, 0x00, 0x01, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_1, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(tanmone_uart_write_timeout)));
    uint8_t buf_read[7];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_1, buf_read, length_read, pdMS_TO_TICKS(tanmone_uart_read_timeout));
    uint16_t received_crc = (buf_read[5] << 8) | buf_read[6];
    uint16_t expected_crc = tanmone_uart_CRC16(buf_read, length_read - 2);
    // Minimum response length check:
    if (length_read < 5) return false; // response length shorter than address (1) + functioncode (1) + bytecount (1) + crc (2)
    // Address check:
    else if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == (func_ | tanmone_exception_func_)) return false; // 0x83
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else { 
        *data = (buf_read[3] << 8) | buf_read[4];
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_1)); 
} 

/*Function code 0x03, Starting address 0x0000, Quantity of registers 0x0002
Data presentation: two decimal places from unsigned short 0 to 1400; one decimal place from short -100 to 1300,
Range: 0.00 to 14.00 pH; -10.0 to 130.0 degC*/
bool tanmone_uart_readBatch(const uint8_t address, uint16_t *data1, int16_t *data2) {
    uint8_t func_ = 0x03;
    uint8_t msg_[] = {address, func_, 0x00, 0x00, 0x00, 0x02};
	uint16_t crc_ = tanmone_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[8] = {address, func_, 0x00, 0x00, 0x00, 0x02, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_1, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(tanmone_uart_write_timeout)));
    uint8_t buf_read[9];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_1, buf_read, length_read, pdMS_TO_TICKS(tanmone_uart_read_timeout));
    uint16_t received_crc = (buf_read[7] << 8) | buf_read[8];
    uint16_t expected_crc = tanmone_uart_CRC16(buf_read, length_read - 2);
    // Minimum response length check:
    if (length_read < 5) return false; // response length shorter than address (1) + functioncode (1) + bytecount (1) + crc (2)
    // Address check:
    else if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == (func_ | tanmone_exception_func_)) return false; // 0x83
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else { 
        *data1 = (buf_read[3] << 8) | buf_read[4];
        *data2 = (buf_read[5] << 8) | buf_read[6];
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_1)); 
}

/*Function code 0x03, Starting address 0x0002, Quantity of registers 0x0001
Data presentation: signed integer from short -2000 to 2000
Range: -2000 to 2000 mV*/
bool tanmone_uart_readORP (const uint8_t address, int16_t *data) { 
    uint8_t func_ = 0x03;
    uint8_t msg_[] = {address, func_, 0x00, 0x02, 0x00, 0x01};
	uint16_t crc_ = tanmone_uart_CRC16(msg_, sizeof(msg_));
    uint8_t src_write[8] = {address, func_, 0x00, 0x02, 0x00, 0x01, (uint8_t) ((crc_&0xFF00)>>8), (uint8_t) crc_&0x00FF};
    uart_write_bytes(UART_NUM_1, src_write, sizeof(src_write));
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(tanmone_uart_write_timeout)));
    uint8_t buf_read[7];
    uint32_t length_read = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length_read));
    length_read = uart_read_bytes(UART_NUM_1, buf_read, length_read, pdMS_TO_TICKS(tanmone_uart_read_timeout));
    uint16_t received_crc = (buf_read[5] << 8) | buf_read[6];
    uint16_t expected_crc = tanmone_uart_CRC16(buf_read, length_read - 2);
    // Minimum response length check:
    if (length_read < 5) return false; // response length shorter than address (1) + functioncode (1) + bytecount (1) + crc (2)
    // Address check:
    else if (buf_read[0] != address) return false;
    // Exception response check:
    else if (buf_read[1] == (func_ | tanmone_exception_func_)) return false; // 0x83
    // Function code check:
    else if (buf_read[1] != func_) return false; 
    // CRC check:
    else if (received_crc != expected_crc) return false;
    // Extract valid data
    else { 
        *data = (buf_read[3] << 8) | buf_read[4];
        return true;
    }
    ESP_ERROR_CHECK(uart_flush(UART_NUM_1)); 
} 

uint16_t tanmone_uart_CRC16(uint8_t *puchMsg, uint16_t usDataLen) {
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint16_t uIndex;
    while (usDataLen--) {
        uIndex = uchCRCHi ^ *puchMsg++ ;
        uchCRCHi = uchCRCLo ^ tanmone_uart_auchCRCHi[uIndex] ;
        uchCRCLo = tanmone_uart_auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
}
