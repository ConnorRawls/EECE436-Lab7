/*
 * temperature.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Connor
 */

#ifndef INC_UTILITIES_TEMPERATURE_H_
#define INC_UTILITIES_TEMPERATURE_H_

#include "stdio.h"
#include "stdbool.h"
#include "string.h"

#define INVALID_TEMPERATURE -1
// Due to the steady state of the system taking long periods, the calibration timeout can easily be disabled/enabled for convenience
#define ENABLE_CALIBRATION_TIMEOUT 0
// The calibration timeout is meant to safely run a while loop with an exit condition
#define CALIBRATION_TIMEOUT 60
#define DEFAULT_DUTY_CYCLE 0.2

typedef struct {
	float duty_cycle;
	float temperature;
} CalibrationPoint;

typedef struct {
	float temperature_steady_count_limit;	// Amount of steady reads we need to consecutively hit before being considered steady
	float temperature_steady_count_thresh;	// Allowable difference between readings to be considered steady
	float temperature_steady_count;			// Number of current consecutive steady reads
	float logged_temperature;
	uint8_t number_of_calibration_points;
	uint8_t calibration_index;				// Current index of calibration point array we are testing
	CalibrationPoint* calibration_points;
} TemperatureCalibrator;

typedef struct {
	float temperature_slope;
	float temperature_offset;
} TemperatureCurve;

uint8_t TemperatureCalibrationRoutine(bool is_manual, UART_HandleTypeDef* huart, TemperatureCurve* temperature_curve, TemperatureCalibrator* temperature_calibrator, float max_input_voltage, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim);
void SetTemperature(TemperatureCurve temperature_curve, int temperature, TIM_HandleTypeDef* htim);
float GetTemperature(ADC_HandleTypeDef* hadc, const float max_input_voltage);
float ConvertFToC(float temperature);

#endif /* INC_UTILITIES_TEMPERATURE_H_ */
