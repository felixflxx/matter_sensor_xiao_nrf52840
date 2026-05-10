/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include "lib/core/CHIPError.h"
#include "lib/support/CodeUtils.h"

#include <setup_payload/OnboardingCodesUtil.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>


#ifdef CONFIG_FUEL_GAUGE
#include <zephyr/drivers/fuel_gauge.h>
#endif


#include <app-common/zap-generated/attributes/Accessors.h>

#include <cmath>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

namespace {
	// Set the measurement interval.
	constexpr size_t kMeasurementsIntervalMs = 300000; // 5 Minutes

	// Endpoint and attribute constants for Temperature Measurement
	constexpr uint8_t kTemperatureMeasurementEndpointId = 1;
	constexpr int16_t kTemperatureMeasurementAttributeMaxValue = 0x7fff;
	constexpr int16_t kTemperatureMeasurementAttributeMinValue = 0x954d;
	constexpr int16_t kTemperatureMeasurementAttributeInvalidValue = 0x8000;

	// Endpoint and attribute constants for Relative Humidity Measurement
	constexpr uint8_t kHumidityMeasurementEndpointId = 2;
	constexpr uint16_t kHumidityMeasurementAttributeMaxValue = 10000; // 100.00%
	constexpr uint16_t kHumidityMeasurementAttributeMinValue = 0; // 0.00%
	constexpr uint16_t kHumidityMeasurementAttributeInvalidValue = 0xffff; // Invalid value

	//Endpoint and attribute constants for Pressure Measurement
	constexpr uint8_t kPressureMeasurementEndpointId = 3; // (Meistens ist Pressure auf Endpoint 3)
	constexpr int16_t kPressureMeasurementAttributeMaxValue = 32767;
	constexpr int16_t kPressureMeasurementAttributeMinValue = -32767;
	constexpr int16_t kPressureMeasurementAttributeInvalidValue = 0x8000;

	k_timer sMeasurementsTimer;
	bool sIsMeasurementTimerStarted = false;

	const device *bme280_dev = DEVICE_DT_GET_ONE(bosch_bme280);

	const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios);
	const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_NODELABEL(led1), gpios);

	//ladezeug
	constexpr uint8_t kPowerSourceEndpointId = 0;
	const struct gpio_dt_spec read_bat_enable = GPIO_DT_SPEC_GET(DT_NODELABEL(bat_enable), gpios);
	const struct gpio_dt_spec chg_stat = GPIO_DT_SPEC_GET(DT_NODELABEL(charge_stat), gpios);
	const struct gpio_dt_spec hi_chgn = GPIO_DT_SPEC_GET(DT_NODELABEL(high_charge), gpios);
	const struct adc_dt_spec ain7_bat = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);


	uint32_t buf = 0;

	// Typische Spannungspunkte eines Li-Ion / Li-Po Akkus (in mV)
	static const int voltage_table[] = {
		3300, // 0%   - Akku leer (tiefer sollte man nicht gehen)
		3500, // 5%   - Steiler Abfall am Ende
		3650, // 20%
		3800, // 40%  - Langes, flaches Plateau
		3950, // 70%  - Langes, flaches Plateau
		4100, // 90%
		4200  // 100% - Akku voll geladen
	};

	static const int percent_table[] = {
		0, 5, 20, 40, 70, 90, 100
	};

	#define TABLE_SIZE (sizeof(voltage_table) / sizeof(voltage_table[0]))


	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
		.calibrate = false
	};


} //namespace

int lipo_voltage_to_percent(int voltage_mv) {
    // Limits abfangen (außerhalb der Tabelle)
    if (voltage_mv <= voltage_table[0]) return 0;
    if (voltage_mv >= voltage_table[TABLE_SIZE - 1]) return 100;

    // Passenden Bereich finden und linear interpolieren
    for (int i = 1; i < TABLE_SIZE; i++) {
        if (voltage_mv <= voltage_table[i]) {
            // Lineare Interpolation (y = y0 + (x - x0) * (y1 - y0) / (x1 - x0))
            int dV = voltage_table[i] - voltage_table[i - 1]; // x1 - x0
            int dP = percent_table[i] - percent_table[i - 1]; // y1 - y0
            
            return percent_table[i - 1] + ((voltage_mv - voltage_table[i - 1]) * dP) / dV;
        }
    }
    
    return 100; // Fallback
}

void AppTask::MatterEventHandler(const ChipDeviceEvent *event, intptr_t arg)
{
	// On initial commissioning, start the timer.
	// We use a flag to ensure this only happens once per boot.
	if (event->Type == DeviceEventType::kCommissioningComplete && !sIsMeasurementTimerStarted) {
		LOG_INF("Commissioning complete, starting measurements.");

		// Take an immediate reading by calling UpdateClustersState directly.
		// This is posted to the event queue to ensure it runs in the correct context.
		Nrf::PostTask([] { Instance().UpdateClustersState(); });

		// Start the periodic timer. K_NO_WAIT is NOT used here because we just
		// triggered the first reading manually. The timer will fire after the first interval.
		k_timer_start(&sMeasurementsTimer, K_MSEC(kMeasurementsIntervalMs), K_MSEC(kMeasurementsIntervalMs));
		sIsMeasurementTimerStarted = true;
	}
}

void AppTask::UpdateTemperatureClusterState()
{
	struct sensor_value sTemperature;
	Protocols::InteractionModel::Status status;
	int result = sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &sTemperature);

	if (result == 0) {
		// The MeasuredValue attribute is in 1/100ths of a degree Celsius.
		// First, get the temperature in 1/100ths of a degree.
		int32_t temp_in_hundredths = sTemperature.val1 * 100 + sTemperature.val2 / 10000;
		int16_t newValue;

		// Round to the nearest half degree (i.e., nearest 50 hundredths).
		// This logic correctly handles rounding for both positive and negative temperatures.
		if (temp_in_hundredths >= 0) {
			newValue = static_cast<int16_t>(((temp_in_hundredths + 25) / 50) * 50);
		} else {
			newValue = static_cast<int16_t>(((temp_in_hundredths - 25) / 50) * 50);
		}

		if (newValue > kTemperatureMeasurementAttributeMaxValue || newValue < kTemperatureMeasurementAttributeMinValue) {
			newValue = kTemperatureMeasurementAttributeInvalidValue;
		}
		LOG_DBG("New temperature measurement: %d.%06d *C, rounded attribute value: %d", sTemperature.val1,
			sTemperature.val2, newValue);

		status = Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(kTemperatureMeasurementEndpointId, newValue);
		if (status != Protocols::InteractionModel::Status::Success) {
			LOG_ERR("Updating temperature measurement failed: %x", to_underlying(status));
		}
	} else {
		LOG_ERR("Getting temperature measurement data from BME280 failed with: %d", result);
	}
}

void AppTask::UpdateHumidityClusterState()
{
	struct sensor_value sHumidity;
	Protocols::InteractionModel::Status status;
	int result = sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &sHumidity);

	if (result == 0) {
		// Round the humidity reading to the nearest whole percentage.
		// The sensor_value struct provides the integer part in val1 and the fractional part
		// in val2 (in millionths). We round up if the fractional part is >= 0.5 (500,000 millionths).
		int32_t rounded_humidity_percent = sHumidity.val1;
		if (sHumidity.val2 >= 500000) {
			rounded_humidity_percent++;
		}

		// The Relative Humidity MeasuredValue attribute is in 1/100ths of a percent.
		uint16_t newValue = static_cast<uint16_t>(rounded_humidity_percent * 100);

		LOG_DBG("New humidity measurement: %d.%06d %%RH, rounded to: %d %%, attribute value: %u", sHumidity.val1,
			sHumidity.val2, rounded_humidity_percent, newValue);

		// Validate the reading is within the defined range for the attribute.
		if (newValue > kHumidityMeasurementAttributeMaxValue || newValue < kHumidityMeasurementAttributeMinValue) {
			newValue = kHumidityMeasurementAttributeInvalidValue;
			LOG_WRN("Humidity value out of range, setting to invalid.");
		}

		status = Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(kHumidityMeasurementEndpointId, newValue);
		if (status != Protocols::InteractionModel::Status::Success) {
			LOG_ERR("Updating humidity measurement failed: %x", to_underlying(status));
		}
	} else {
		LOG_ERR("Getting humidity measurement data from BME280 failed with: %d", result);
	}
}


void AppTask::UpdatePressureClusterState()
{
    struct sensor_value sPressure;
    Protocols::InteractionModel::Status status;
    
    // Zephyr holt den Luftdruck aus dem BME280 RAM
    int result = sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &sPressure);

    if (result == 0) {
        // Umrechnung für Matter: (kPa * 10) + (Millionstel kPa / 100.000)
        int16_t newValue = static_cast<int16_t>(sPressure.val1 * 10 + sPressure.val2 / 100000);

        LOG_DBG("New pressure: %d.%06d kPa, Matter attribute: %d", sPressure.val1, sPressure.val2, newValue);

        if (newValue > kPressureMeasurementAttributeMaxValue || newValue < kPressureMeasurementAttributeMinValue) {
            newValue = kPressureMeasurementAttributeInvalidValue;
        }

        status = Clusters::PressureMeasurement::Attributes::MeasuredValue::Set(kPressureMeasurementEndpointId, newValue);
        if (status != Protocols::InteractionModel::Status::Success) {
            LOG_ERR("Updating pressure measurement failed: %x", to_underlying(status));
        }
    } else {
        LOG_ERR("Getting pressure from BME280 failed with: %d", result);
    }
}



void AppTask::UpdateBatteryClusterState()
{
	LOG_DBG("Batterie Update Funktion aufgerufen!");
	Protocols::InteractionModel::Status status;

	// 1. Get Battery Voltage			DONE
	int32_t val_mv;
	int err = adc_read_dt(&ain7_bat, &sequence);
			if (err < 0) {
				LOG_ERR("Could not read (%d)\n", err);
				chip::System::MapErrorZephyr(-ENODEV);
				return;
			}
	if (ain7_bat.channel_cfg.differential) {
				val_mv = (int32_t)((int16_t)buf);
	} else {
		val_mv = (int32_t)buf;
	}
	LOG_DBG("%"PRId32, val_mv);
	err = adc_raw_to_millivolts_dt(&ain7_bat,
						&val_mv);
	/* conversion to mV may not be supported, skip if not */
	if (err < 0) {
		LOG_ERR(" (value in mV not available)\n");
	}
	int32_t bat_mv = val_mv * 1510 / 510;
	status = Clusters::PowerSource::Attributes::BatVoltage::Set(kPowerSourceEndpointId, bat_mv);

	if (status != Protocols::InteractionModel::Status::Success) {
		LOG_ERR("Updating BatVoltage failed: %x", to_underlying(status));
		// Set to "unknown" (0xFFFFFFFF) on failure
		status = Clusters::PowerSource::Attributes::BatVoltage::Set(kPowerSourceEndpointId, 0xFFFFFFFF);
		if (status != Protocols::InteractionModel::Status::Success) {
		LOG_ERR("Setting BatVoltage to 'unknown' failed: %x", to_underlying(status));
		}
	}
	
	

	// 2. Get Battery Percentage

	int bat_prozent = lipo_voltage_to_percent(bat_mv);

	uint8_t matter_pct = static_cast<uint8_t>(bat_prozent * 2);

	LOG_DBG("New battery percentage: %d", bat_prozent);

	status = Clusters::PowerSource::Attributes::BatPercentRemaining::Set(kPowerSourceEndpointId, matter_pct);
	if (status != Protocols::InteractionModel::Status::Success) {
		LOG_ERR("Updating BatPercentRemaining failed: %x", to_underlying(status));
	}

	// 3. Get Charging State		DONE
	// The driver returns 'true' if charging
	int val = gpio_pin_get_dt(&chg_stat);
		Clusters::PowerSource::BatChargeStateEnum charge_state;

	if (val == 0) {
		// If time to full is > 0, we are charging
		charge_state = Clusters::PowerSource::BatChargeStateEnum::kIsCharging;
	} else if (val == 1) {
		// Otherwise, we are not charging (or are full)
		charge_state = Clusters::PowerSource::BatChargeStateEnum::kIsNotCharging;
	}
	// NOTE: When Reading PIN directly "battery full" state is not available.
	// kNotCharging is the best fit for discharging or full.

	LOG_DBG("Battery state is %u", (uint8_t)charge_state);
	status = Clusters::PowerSource::Attributes::BatChargeState::Set(kPowerSourceEndpointId, charge_state);
	if (status != Protocols::InteractionModel::Status::Success) {
		LOG_ERR("Updating BatChargeState failed: %x", to_underlying(status));
	}
}

void AppTask::MeasurementsTimerHandler()
{
	Instance().UpdateClustersState();
}

void AppTask::UpdateClustersState()
{
	UpdateBatteryClusterState();
	
	// Fetch a new sample from the sensor. This updates all channels.
	const int result_bme = sensor_sample_fetch(bme280_dev);

	if (result_bme == 0) {
		// Update both clusters with the new data.
		UpdateTemperatureClusterState();
		UpdateHumidityClusterState();
		UpdatePressureClusterState();
	} else {
		LOG_ERR("Fetching data from BME280 sensor failed with: %d", result_bme);
	}
}

CHIP_ERROR AppTask::Init()
{
	//led config
	if (device_is_ready(led_blue.port)) {
        gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);
    }
	if (device_is_ready(led_green.port)) {
        gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    }

	//chg config
	if (device_is_ready(read_bat_enable.port)) {
        gpio_pin_configure_dt(&read_bat_enable, GPIO_OUTPUT_INACTIVE);
		gpio_pin_set_dt(&read_bat_enable, 0);
    }
	if (device_is_ready(chg_stat.port)) {
        gpio_pin_configure_dt(&chg_stat, GPIO_INPUT);
    }
	if (device_is_ready(hi_chgn.port)) {
        gpio_pin_configure_dt(&hi_chgn, GPIO_OUTPUT_INACTIVE);
		gpio_pin_set_dt(&hi_chgn, 0);
    }

	//adc config
	if (!adc_is_ready_dt(&ain7_bat)) {
			LOG_ERR("ADC controller is not ready");
			return chip::System::MapErrorZephyr(-ENODEV);
		}
	int err = adc_channel_setup_dt(&ain7_bat);
		if (err < 0) {
			LOG_ERR("Could not setup channel (%d)\n", err);
			return chip::System::MapErrorZephyr(-EINVAL);
		}
	err = adc_sequence_init_dt(&ain7_bat, &sequence);
    if (err < 0) {
        LOG_ERR("Could not init sequence %d", err);
        return chip::System::MapErrorZephyr(-EINVAL);
    }
	
	
	/* Initialize Matter stack */
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer());

	if (!Nrf::GetBoard().Init()) {
		LOG_ERR("User interface initialization failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	/* Register Matter event handler that controls the connectivity status LED based on the captured Matter network
	 * state. */
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));

	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(MatterEventHandler, 0));

	if (!device_is_ready(bme280_dev)) {
		LOG_ERR("SHT4X sensor device not ready");
		gpio_pin_set_dt(&led_blue, 1);
		k_sleep(K_MSEC(2000));
		return chip::System::MapErrorZephyr(-ENODEV);
	}

	gpio_pin_set_dt(&led_green, 1);

	k_timer_init(&sMeasurementsTimer, [](k_timer *) { Nrf::PostTask([] { MeasurementsTimerHandler(); }); }, nullptr);

	ReturnErrorOnFailure(Nrf::Matter::StartServer());

	// On reboot, if the device is already commissioned, start the timer directly.
	if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0 && !sIsMeasurementTimerStarted) {
		LOG_INF("Device is already commissioned on boot, starting measurements.");
		// Use K_NO_WAIT to get an immediate reading on reboot.
		k_timer_start(&sMeasurementsTimer, K_NO_WAIT, K_MSEC(kMeasurementsIntervalMs));
		sIsMeasurementTimerStarted = true;
		
		AppTask::Instance().UpdateClustersState();
		gpio_pin_set_dt(&led_green, 0);
	}

	return CHIP_NO_ERROR;
}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
