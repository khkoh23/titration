#include "ae_ep_uart.h"

uint8_t ae_ep_uart_readDeviceId (void) { // Function code 0x01
    return 0x01;
}

void ae_ep_uart_writeDeviceId (const uint8_t id, const uint8_t msg) { // Function code 0x02

}

uint16_t ae_ep_uart_readSetVolume (const uint8_t id) { // Function code 0x03
    return 0x0046;
}

void ae_ep_uart_writeSetVolume (const uint8_t id, const uint16_t msg) { // Function code 0x04

}

uint16_t ae_ep_uart_readPipetteSpeed (const uint8_t id) { // Function code 0x05
    return 0x0305;
}

void ae_ep_uart_writePipetteSpeed (const uint8_t id, const uint16_t msg) { // Function code 0x06

}

void ae_ep_uart_cmdAspire (const uint8_t id) { // Function code 0x07

}

void ae_ep_uart_cmdDispense (const uint8_t id) { // Function code 0x08

}

void ae_ep_uart_cmdDispenseStepVolume (const uint8_t id, const uint16_t) { // Function code 0x09

}

void ae_ep_uart_cmdZero (const uint8_t id) { // Function code 0x0A

}

uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen) {
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    uint16_t usIndex;
    while (_usLen--) {
        usIndex = ucCRCHi ^ *_pBuf++;
        ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
        ucCRCLo = s_CRCLo[usIndex];
    }
    return ((uint16_t) ucCRCLo << 8 | ucCRCHi);
}

