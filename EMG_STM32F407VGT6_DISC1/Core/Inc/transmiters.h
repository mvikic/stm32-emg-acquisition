#ifndef USART_TRANSMIT_H
#define USART_TRANSMIT_H

#include "main.h"
#include "stm32f4xx_hal.h"

// Function to send an integer with newline over UART
void send_number_with_newline(UART_HandleTypeDef *huart, uint16_t num);

// Function to send a float with newline over UART
void send_float_with_newline(UART_HandleTypeDef *huart, float num);

#endif // USART_TRANSMIT_H