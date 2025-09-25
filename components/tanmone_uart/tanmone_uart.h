#ifndef _TANMONE_UART_
#define _TANMONE_UART_

#include "driver/uart.h"

float tanmone_uart_readpH (void); // two decimal places from unsigned short
float tanmone_uart_readTemperature (void); // one decimal place from short
int16_t tanmone_uart_readORP (void); // signed integer from short

uint16_t tanmone_uart_ModRTU_CRC(uint8_t * _pBuf, uint16_t _usLen); // Modbus RTU CRC calculation

#endif // _TANMONE_UART_