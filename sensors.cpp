#include <Arduino.h>
#include "sensors.h"
#include "bme280.h"
#include "pendulum.h"
#include "settings.h"
#include "matrix_drive.h"

BME280 bme280;

static bool sensors_get_flag = false;
static bool sensors_change_contrast_flag = false;
bme280_result_t bme280_result;

static constexpr int num_contrast_steps = 20;
#define DEFAULT_CONTRAST F("20")
static uint8_t contrasts[num_contrast_steps];
static constexpr uint8_t max_contrast = 63;
static constexpr uint8_t min_contrast = 0;
static constexpr int brightness_to_index_factor = 43;
static constexpr int brightness_index_min_change = 2; //!< the index of brightness must be changed at least this amount to change contrast
static uint16_t current_brightness = 0; //!< current environment brightness
static uint8_t brightness_get_delay = 0; // stop brightness sensing during this flag raises
static bool contrasts_dirty = false; //!< whether the contrast settings is dirty, need to write to settings fs
static bool contrast_always_max = false; //!< whether not to allow automatic change of contrast or not

static uint8_t target_contrast_value = max_contrast; //!< targetting contrast value
static uint8_t current_contrast_value = 0; //!< current contrast value
static int last_triggered_brightness_index = -100; //!< brightness index as of last contrast changing

static void sensors_bme280_get()
{
	if(!bme280.available()) return;
	double temperature, humidity, pressure;
	uint8_t measuring, im_update;
	bme280.getData(&temperature, &humidity, &pressure);

	bme280_result.temp_10 = temperature * 10;
	bme280_result.humidity = humidity;
	bme280_result.pressure = pressure;
}

void sensors_set_contrast_always_max(bool b) { contrast_always_max = b; }

//! change LED matrix's contrast to targetting value
static void sensors_change_contrast()
{
	if(contrast_always_max)
	{
		led_set_contrast(max_contrast);
	}
	else
	{
		if(current_contrast_value < target_contrast_value)
			++current_contrast_value;
		else if(current_contrast_value > target_contrast_value)
			--current_contrast_value;
		led_set_contrast(current_contrast_value);
	}
}

static void sensors_get_env_brightness()
{
	if(brightness_get_delay)
	{
		--brightness_get_delay;
		return;
	}
	current_brightness = analogRead(0);

	// check if current brightness changes enough to trigger contrast change
	int index = current_brightness / brightness_to_index_factor;
	if(index >= num_contrast_steps) index = num_contrast_steps - 1;
	if(index == 0 || index == num_contrast_steps - 1 ||
		index - last_triggered_brightness_index >= brightness_index_min_change ||
		last_triggered_brightness_index - index >= brightness_index_min_change)
	{
		// trigger contrast change
		int contrast = contrasts[index];
		if(index <= num_contrast_steps - 2)
		{
			// interpolate
			contrast += 
				(contrasts[index + 1] - contrasts[index]) *
					(current_brightness % brightness_to_index_factor) /
					brightness_to_index_factor;
		}

		target_contrast_value = contrast; // set target contrast
	}
}

static void sensors_write_contrasts_settings()
{
	if(!contrasts_dirty) return;
	contrasts_dirty = false;

	string_vector vec;
	for(int i = 0; i < num_contrast_steps; ++i) vec.push_back(String((int)contrasts[i]));
	settings_write_vector(F("sensors_contrasts"), vec);
}


static void sensors_write_default_contrasts_settings(string_vector &vec)
{
	vec.clear();
	for(int i = 0; i < num_contrast_steps; ++i) vec.push_back(String(DEFAULT_CONTRAST));
	settings_write_vector(F("sensors_contrasts"), vec, SETTINGS_OVERWRITE);
}

//! change current contrast; dir=-1: decrease, dir=1: increase
void sensors_change_current_contrast(int dir)
{
	brightness_get_delay = 1; // stop brightness sensing for a while the human changes the value

	int sens = current_brightness;

	// sens value range is approx. 0 <= sens <= 860
	int index = sens / brightness_to_index_factor;
	if(index >= num_contrast_steps) index = num_contrast_steps - 1;

	int value = contrasts[index];
	value += dir;
	if(value < min_contrast) value = min_contrast;
	if(value > max_contrast) value = max_contrast;
	contrasts[index] = value;
	contrasts_dirty = true; // write these settings at delayed timing

	// contrast value of the index below current index must be below the value of the index
	for(int i = 0; i < index; ++i) if(contrasts[i] > value) contrasts[i] = value;

	// and so about above index
	for(int i = index + 1; i < num_contrast_steps; ++i) if(contrasts[i] < value) contrasts[i] = value;

	// change current contrast
	target_contrast_value = value;
	current_contrast_value = value;
	led_set_contrast(current_contrast_value);
}


void sensors_init()
{
	bme280.begin();
	bme280.setMode(BME280_MODE_NORMAL, BME280_TSB_1000MS, BME280_OSRS_x1,
		BME280_OSRS_x1, BME280_OSRS_x1, BME280_FILTER_OFF);
	sensors_bme280_get();

	// prepare initial value for contrasts
	string_vector vec;
	settings_read_vector(F("sensors_contrasts"), vec);

	// is vector valid?
	if(vec.size() != 20)
		sensors_write_default_contrasts_settings(vec);

	// convert string vector to integer vector
	for(int i = 0; i < num_contrast_steps; ++i) contrasts[i] = vec[i].toInt();
}

void sensors_raise_flag() { sensors_get_flag = true; }
static pendulum_t sensors_pendulum_1(&sensors_raise_flag, 10000);

void sensors_raise_change_contrast_flag() { sensors_change_contrast_flag = true; }
static pendulum_t sensors_pendulum_2(&sensors_raise_change_contrast_flag, 200);

void sensors_check()
{
	if(sensors_get_flag)
	{
		sensors_get_flag = false;
		sensors_bme280_get();
		sensors_get_env_brightness();

		// write contrast settings only this interval to reduce stress on flash write
		sensors_write_contrasts_settings();  
	}

	if(sensors_change_contrast_flag)
	{
		sensors_change_contrast_flag = false;

		sensors_change_contrast();
	}
}


