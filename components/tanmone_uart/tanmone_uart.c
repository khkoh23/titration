#include "tanmone_uart.h"

float tanmone_uart_readpH (void) { // Server address 0x00, Function code 0x03
    // two decimal places from unsigned short
    return 6.86; // pH 6.86
} 

float tanmone_uart_readTemperature (void) { // Server address 0x01, Function code 0x03
    // one decimal place from short
    return 25.0; //25.0C
} 

int16_t tanmone_uart_readORP (void) { // Server address 0x02, Function code 0x03
    // signed integer from short
    return 300; // 300mV
} 

uint16_t tanmone_uart_ModRTU_CRC(uint8_t *_pBuf, uint16_t _usLen) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < _usLen; pos++) {
        crc ^= (uint16_t)_pBuf[pos]; // XOR byte into LSB of CRC
        for (int i = 8; i != 0; i--) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1; // Shift right
                crc ^= 0xA001; // XOR with polynomial
            } 
            else { // Else LSB is not set
                crc >>= 1; // Just shift right
            }
        }
    }
    // No byte swap needed inside function if handling little-endian architecture.
    // The CRC bytes are swapped and appended later.
  return crc;
}
