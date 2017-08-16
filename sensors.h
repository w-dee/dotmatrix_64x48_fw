#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

struct bme280_result_t
{
	int temp_10; // temperature in deg C, 10 multiplied
	int humidity; // humidity in RH%
	int pressure; // atmosphere pressure in hPa
};

extern bme280_result_t bme280_result;
void sensors_set_contrast_always_max(bool b);
void sensors_change_current_contrast(int dir);
void sensors_init();
void sensors_check();

#endif

