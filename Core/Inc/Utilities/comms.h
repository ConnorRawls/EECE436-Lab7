/*
 * comms.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Connor
 */

#ifndef INC_UTILITIES_COMMS_H_
#define INC_UTILITIES_COMMS_H_

#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stm32l4xx_hal.h"

#define ASCII_NUMBER_OFFSET 48
#define ERROR_INVALID_INPUT 100
#define TX_BUFF_SIZE 100
#define RX_BUFF_SIZE 100
#define RX_MAX_DIGITS 3

void OutputToTerminal(UART_HandleTypeDef* huart, const char* string);
int ReadValueFromTerminal(UART_HandleTypeDef* huart);
bool IsUserInputComplete(uint8_t* input_string, uint8_t string_length);
uint8_t ParseUserInput(uint8_t* input_string, uint8_t string_length);

#endif /* INC_UTILITIES_COMMS_H_ */
