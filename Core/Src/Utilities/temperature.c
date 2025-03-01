/*
 * temperature.c
 *
 *  Created on: Feb 22, 2025
 *      Author: Connor
 */

#include "comms.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "Utilities/temperature.h"

// =====================================================================================
// Private Function Prototypes
// =====================================================================================
float ConvertCountsToVoltage(uint16_t counts, float max_input_voltage);
float ConvertVoltageToTemperature(float voltage);
void AdjustDutyCycle(float duty_cycle, TIM_HandleTypeDef* htim);
void FormTemperatureCurve(TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator);
uint8_t ManualTemperatureCalibration(UART_HandleTypeDef* huart, TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator, float max_input_voltage, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim);
uint8_t AutoTemperatureCalibration(UART_HandleTypeDef* huart, TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator, float max_input_voltage, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim);

// =====================================================================================
// Public Functions
// =====================================================================================
uint8_t TemperatureCalibrationRoutine(bool is_manual, UART_HandleTypeDef* huart, TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator, float max_input_voltage, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim)
{
	uint8_t return_code = 0;

	if (is_manual) return_code = ManualTemperatureCalibration(huart, temperature_curve, temperature_calibrator, max_input_voltage, hadc, htim);
	else return_code = AutoTemperatureCalibration(huart, temperature_curve, temperature_calibrator, max_input_voltage, hadc, htim);

	return return_code;
}

void SetTemperature(TemperatureCurve temperature_curve, int temperature, TIM_HandleTypeDef* htim)
{
	if (temperature == INVALID_TEMPERATURE) return;

	float duty_cycle = (temperature_curve.temperature_slope * temperature) + temperature_curve.temperature_offset;

	AdjustDutyCycle(duty_cycle, htim);
}

float GetTemperature(ADC_HandleTypeDef* hadc, const float max_input_voltage)
{
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 100);
	uint32_t raw_value = HAL_ADC_GetValue(hadc);
	float voltage = ConvertCountsToVoltage(raw_value, max_input_voltage);
	HAL_ADC_Stop(hadc);
	return ConvertVoltageToTemperature(voltage);
}

float ConvertFToC(float temperature)
{
	return (temperature - 32.0) * (5.0 / 9.0);
}

// =====================================================================================
// Private Functions
// =====================================================================================
uint8_t ManualTemperatureCalibration(UART_HandleTypeDef* huart, TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator, float max_input_voltage, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim)
{
	uint8_t str_buff[64] = { '\0' };

	while (true) {
		float current_temperature = GetTemperature(hadc, max_input_voltage);

		sprintf((char*)str_buff, "Current temperature: %3.2f F (%3.2f C)\r\n", current_temperature, ConvertFToC(current_temperature));
		OutputToTerminal(huart, (char*)str_buff);
		memset(str_buff, '\0', sizeof(str_buff));

		int user_input = ReadValueFromTerminal(huart);
		// Stall until user wants us to continue
		while (user_input != '\0') {

		}

		// If valid user input is found, set the new temperature
		switch (user_input) {
			case 'x':
				return 1;
				break;
			case ''
		}
		if (user_input != 'x') {
		  sprintf((char*)str_buff, "Setting temperature: %d F\r\n", desired_temperature);
		  OutputToTerminal(&huart2, (char*)str_buff);
		  SetTemperature(temperature_curve, desired_temperature, &htim2);
		  memset(str_buff, '\0', sizeof(str_buff));
		}

		FormTemperatureCurve(temperature_curve, temperature_calibrator);
		AdjustDutyCycle(DEFAULT_DUTY_CYCLE, htim);
	}

	return 1;
}

uint8_t AutoTemperatureCalibration(UART_HandleTypeDef* huart, TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator, float max_input_voltage, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim)
{
	if (ENABLE_CALIBRATION_TIMEOUT) {
		uint16_t cycle_counter = 0;

		while (cycle_counter <= CALIBRATION_TIMEOUT) {
			float current_temperature = GetTemperature(hadc, max_input_voltage);

			// If this is the first cycle, log current temperature
			if (temperature_calibrator->logged_temperature == 0) {
				temperature_calibrator->logged_temperature = current_temperature;
			}
			// Else if the current temperature reading is still steady, increment steady count
			else if (abs(temperature_calibrator->logged_temperature - current_temperature) < temperature_calibrator->temperature_steady_count_thresh) {
				temperature_calibrator->temperature_steady_count++;
			}
			// Else unsteady condition, reset temperature logging
			else {
				temperature_calibrator->logged_temperature = current_temperature;
				temperature_calibrator->temperature_steady_count = 0;
			}

			// If we have enough steady state temperature data, move on to the next PWM duty cycle and read
			if (temperature_calibrator->temperature_steady_count >= temperature_calibrator->temperature_steady_count_limit) {
			  temperature_calibrator->calibration_points[temperature_calibrator->calibration_index].temperature = current_temperature;

			  if (temperature_calibrator->calibration_index + 1 >= temperature_calibrator->number_of_calibration_points) {
				  FormTemperatureCurve(temperature_curve, temperature_calibrator);
				  AdjustDutyCycle(DEFAULT_DUTY_CYCLE, htim);
				  return 1;
			  }
			  else {
				  temperature_calibrator->calibration_index++;
				  temperature_calibrator->temperature_steady_count = 0;
				  AdjustDutyCycle(temperature_calibrator->calibration_points[temperature_calibrator->calibration_index].duty_cycle, htim);
			  }
			}

			cycle_counter++;
			HAL_Delay(1000);
		}
	}
	else {
		float current_temperature = GetTemperature(hadc, max_input_voltage);

		// If this is the first cycle, log current temperature
		if (temperature_calibrator->logged_temperature == 0) {
			temperature_calibrator->logged_temperature = current_temperature;
		}
		// Else if the current temperature reading is still steady, increment steady count
		else if (abs(temperature_calibrator->logged_temperature - current_temperature) < temperature_calibrator->temperature_steady_count_thresh) {
			temperature_calibrator->temperature_steady_count++;
		}
		// Else unsteady condition, reset temperature logging
		else {
			temperature_calibrator->logged_temperature = current_temperature;
			temperature_calibrator->temperature_steady_count = 0;
		}

		// If we have enough steady state temperature data, move on to the next PWM duty cycle and read
		if (temperature_calibrator->temperature_steady_count >= temperature_calibrator->temperature_steady_count_limit) {
		  temperature_calibrator->calibration_points[temperature_calibrator->calibration_index].temperature = current_temperature;

		  if (temperature_calibrator->calibration_index + 1 >= temperature_calibrator->number_of_calibration_points) {
			  FormTemperatureCurve(temperature_curve, temperature_calibrator);
			  AdjustDutyCycle(DEFAULT_DUTY_CYCLE, htim);
			  return 1;
		  }
		  else {
			  temperature_calibrator->calibration_index++;
			  temperature_calibrator->temperature_steady_count = 0;
			  AdjustDutyCycle(temperature_calibrator->calibration_points[temperature_calibrator->calibration_index].duty_cycle, htim);
		  }
		}

		HAL_Delay(1000);
	}

	return 0;
}

float ConvertCountsToVoltage(uint16_t counts, float max_input_voltage)
{
	float count_ratio = (float)counts / 0xFFF;
	float voltage = count_ratio * max_input_voltage;
	return voltage;
}

float ConvertVoltageToTemperature(float voltage)
{
	float temperature = ((voltage - 1.25) / 0.005) + 25.0;
	return (temperature * (9.0 / 5.0)) + 32.0;
}

void AdjustDutyCycle(float duty_cycle, TIM_HandleTypeDef* htim)
{
	uint16_t pulse_width = (uint16_t)(duty_cycle * htim->Init.Period); // Casting as int ensures we floor the result

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse_width);
}

void FormTemperatureCurve(TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator)
{
	CalibrationPoint calibration_point1 = temperature_calibrator->calibration_points[0];
	CalibrationPoint calibration_point2 = temperature_calibrator->calibration_points[1];

	float slope = (calibration_point2.duty_cycle - calibration_point1.duty_cycle) / (calibration_point2.temperature - calibration_point1.temperature);
	float offset = calibration_point1.temperature - (slope * calibration_point1.duty_cycle);

	temperature_curve->temperature_slope = slope;
	temperature_curve->temperature_offset = offset;

	return;
}
