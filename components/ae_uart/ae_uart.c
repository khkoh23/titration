#include "ae_uart.h"

uint8_t ae_uart_readDeviceId (void) { // Function code 0x01
    return 0x01;
}

void ae_uart_writeDeviceId (const uint8_t id, const uint8_t msg) { // Function code 0x02

}

uint16_t ae_uart_readSetVolume (const uint8_t id) { // Function code 0x03
    return 0x0046; // 7.0uL in 200.0uL
}

void ae_uart_writeSetVolume (const uint8_t id, const uint16_t msg) { // Function code 0x04

}

uint16_t ae_uart_readPipetteSpeed (const uint8_t id) { // Function code 0x05
    return 0x0305; // aspire speed = 3, dispense speed = 5
}

void ae_uart_writePipetteSpeed (const uint8_t id, const uint16_t msg) { // Function code 0x06

}

void ae_uart_cmdAspire (const uint8_t id) { // Function code 0x07
    uint8_t msg_[] = {id, 0x07};
	uint16_t len_ = sizeof(msg_);
	uint16_t crc_ = ae_uart_CRC16_Modbus(msg_, len_);
    uint8_t src_[4] = {id, 0x07, (uint8_t) crc_&0x00FF, (uint8_t) ((crc_&0xFF00)>>8)};
    size_t size_ = sizeof(src_);
    uart_write_bytes(UART_NUM_2, src_, size_);
}

void ae_uart_cmdDispense (const uint8_t id) { // Function code 0x08
    uint8_t msg_[] = {id, 0x08};
	uint16_t len_ = sizeof(msg_);
	uint16_t crc_ = ae_uart_CRC16_Modbus(msg_, len_);
    uint8_t src_[4] = {id, 0x08, (uint8_t) crc_&0x00FF, (uint8_t) ((crc_&0xFF00)>>8)};
    size_t size_ = sizeof(src_);
    uart_write_bytes(UART_NUM_2, src_, size_);
}

void ae_uart_cmdDispenseStepVolume (const uint8_t id, const uint16_t) { // Function code 0x09

}

void ae_uart_cmdZero (const uint8_t id) { // Function code 0x0A
    uint8_t msg_[] = {id, 0x0A};
	uint16_t len_ = sizeof(msg_);
	uint16_t crc_ = ae_uart_CRC16_Modbus(msg_, len_);
    uint8_t src_[4] = {id, 0x0A, (uint8_t) crc_&0x00FF, (uint8_t) ((crc_&0xFF00)>>8)};
    size_t size_ = sizeof(src_);
    uart_write_bytes(UART_NUM_2, src_, size_);
}

uint16_t ae_uart_CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen) {
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    uint16_t usIndex;
    while (_usLen--) {
        usIndex = ucCRCHi ^ *_pBuf++;
        ucCRCHi = ucCRCLo ^ ae_uart_s_CRCHi[usIndex];
        ucCRCLo = ae_uart_s_CRCLo[usIndex];
    }
    return ((uint16_t) ucCRCLo << 8 | ucCRCHi);
}

