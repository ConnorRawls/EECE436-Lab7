/*
 * comms.c
 *
 *  Created on: Feb 22, 2025
 *      Author: Connor
 */

#include "Utilities/comms.h"
#include "math.h"
#include "stm32l4xx_hal.h"

char* usage_string = \
	  "\r\n\r\nUSAGE:\r\n"
	  "Enter in a number between 0 and 99. The LED will blink this many times.\r\n" \
	  "Press ENTER to submit your value.\r\n";
char* idle_string = "\r\nNumber: ";
char* value_received_string = "\r\n\r\nReceived value: %d\r\n";
char* blink_count_string = "\r\nLED blink count: %d\r\n";
char* leaving_loop_string = "\r\nExit code detected. Leaving loop...\r\n";
char* exit_wait_string = "\r\nPress 'x' to stop.\r\n";
char* error_string = "\r\n\r\nError: Invalid input.\r\n";
uint8_t Tx_buff[TX_BUFF_SIZE] = {'\0'};
uint8_t Rx_data[RX_BUFF_SIZE] = {'\0'};
uint8_t Rx_offset = 0; // Keep track of which digit we're on

bool IsUserInputComplete(uint8_t* input_string, uint8_t string_length);
uint8_t ParseUserInput(uint8_t* input_string, uint8_t string_length);

void OutputToTerminal(UART_HandleTypeDef* huart, const char* string)
{
	sprintf((char*)Tx_buff, string);
	HAL_UART_Transmit(huart, Tx_buff, sizeof(Tx_buff), 100);
	memset(Tx_buff, '\0', sizeof(Tx_buff));
}

int ReadValueFromTerminal(UART_HandleTypeDef* huart)
{
	HAL_UART_Receive(huart, Rx_data + Rx_offset, 1, 100);

	// If input found, move over a digit
	if (Rx_data[Rx_offset] != '\0') Rx_offset++;

	// Determine if user string is complete
	if (IsUserInputComplete(Rx_data, RX_BUFF_SIZE)) {
	  // Parse value out of user input
	  uint16_t value = ParseUserInput(Rx_data, Rx_offset);

	  Rx_offset = 0;

	  // Invalid value found
	  if (value == ERROR_INVALID_INPUT) {
		  OutputToTerminal(huart, error_string);
		  OutputToTerminal(huart, idle_string);
		  memset(Rx_data, '\0', sizeof(Rx_data));
		  value = -1;
	  }

	  // Reset communication lines
	  OutputToTerminal(huart, value_received_string);
	  OutputToTerminal(huart, exit_wait_string);
	  OutputToTerminal(huart, idle_string);
	  memset(Rx_data, '\0', sizeof(Rx_data));

	  return value;
	}
	// Some invalid value has been found, reset
	else if (Rx_offset >= RX_MAX_DIGITS + 1) {
	  OutputToTerminal(huart, error_string);
	  OutputToTerminal(huart, idle_string);
	  memset(Rx_data, '\0', sizeof(Rx_data));
	  Rx_offset = 0;
	}

	return -1;
}

// Searches string for a carriage return
bool IsUserInputComplete(uint8_t* input_string, uint8_t string_length)
{
	for (int i = 0; i < string_length; i++) {
		if (input_string[i] == '\r') return true;
	}

	return false;
}

// Converts user input into a uint8_t value
uint8_t ParseUserInput(uint8_t* input_string, uint8_t string_length)
{
	uint8_t input_value = 0; // Return value
	uint8_t digit_offset = 0; // Digit offset tracker

	// Go through input array backwards
	for (int i = (string_length - 1); i >= 0; i--) {
		if (input_string[i] == '\r') continue;
		if (input_string[i] < '0' || input_string[i] > '9') return ERROR_INVALID_INPUT;
		input_value += (input_string[i] - ASCII_NUMBER_OFFSET) * pow(10, digit_offset);
		digit_offset++;
	}

	return input_value;
}
