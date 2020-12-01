#ifndef SITRONLABS_MCP9808_H
#define SITRONLABS_MCP9808_H

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* C library */
#include <errno.h>

/**
 *
 */
enum mcp9808_resolution {
	MCP9808_RESOLUTION_0_5000 = 0b00, //!< +0.5°C (tCONV = 30 ms typical)
	MCP9808_RESOLUTION_0_2500 = 0b01, //!< +0.25°C (tCONV = 65 ms typical)
	MCP9808_RESOLUTION_0_1250 = 0b10, //!< +0.125°C (tCONV = 130 ms typical)
	MCP9808_RESOLUTION_0_0625 = 0b11, //!< +0.0625°C (power-up default, tCONV = 250 ms typical)
};

/**
 * Alert hysteresis.
 * @see Datasheet section 5.1.1.
 * @see Datasheet section 5.2.2.
 */
enum mcp9808_alert_hysteresis {
	MCP9808_ALERT_HYSTERESIS_0_0 = 0b00, //!< 0°C (power-up default)
	MCP9808_ALERT_HYSTERESIS_1_5 = 0b01, //!< +1.5°C
	MCP9808_ALERT_HYSTERESIS_3_0 = 0b10, //!< +3.0°C
	MCP9808_ALERT_HYSTERESIS_6_0 = 0b11, //!< +6.0°C
};

/**
 * Alert output mode.
 * @see Datasheet section 5.1.1.
 * @see Datasheet section 5.2.3.
 */
enum mcp9808_alert_output_mode {
};

/**
 * Alert output polarity.
 * @see Datasheet section 5.1.1.
 * @see Datasheet section 5.2.3.
 */
enum mcp9808_alert_output_polarity {
	MCP9808_ALERT_OUTPUT_POLARITY_LOW = 0,  //!< Active-low (power-up default; pull-up resistor required)
	MCP9808_ALERT_OUTPUT_POLARITY_HIGH = 1, //!< Active-high
};

/**
 *
 */
enum mcp9808_registers {
	MCP9808_REG_CONFIG = 0x01,         //!< Configuration.
	MCP9808_REG_ALERT_UPPER = 0x02,    //!< Alert upper threshold.
	MCP9808_REG_ALERT_LOWER = 0x03,    //!< Alert lower threshold.
	MCP9808_REG_ALERT_CRITICAL = 0x04, //!< Alert critical threshold.
	MCP9808_REG_TEMPERATURE = 0x05,    //!< Measured ambient temperature
	MCP9808_REG_MANUF_ID = 0x06,       //!< Manufacturer id.
	MCP9808_REG_DEVICE_ID = 0x07,      //!< Device id and revision.
	MCP9808_REG_RESOLUTION = 0x08,     //!< Temperature resolution.
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            MCP9808 Temp Sensor
 */
class mcp9808 {
public:
	mcp9808();
	void setup();
	void setup(TwoWire *i2c_library);
	void setup(const uint8_t i2c_address);
	void setup(const uint8_t i2c_address, TwoWire *i2c_library);
	bool detect(void);
	int temperature_read_celsius(float * const temperature);
	int resolution_get(enum mcp9808_resolution * const resolution);
	int resolution_set(const enum mcp9808_resolution resolution);
	int alert_hysteresis_set(const enum mcp9808_alert_hysteresis hysteresis);
	int alert_threshold_lower_set(const float temperature);
	int alert_threshold_upper_set(const float temperature);
	int alert_threshold_critical_set(const float temperature);
	int alert_output_set(const boolean enabed, const enum mcp9808_alert_output_mode mode, const enum mcp9808_alert_output_polarity polarity);
	int interrupt_clear(void);
	int shutdown_set(const boolean shutdown);
private:
	TwoWire *m_i2c_library;
	uint8_t m_i2c_address;
	int m_register_read8(const enum mcp9808_registers address, uint8_t * const value);
	int m_register_write8(const enum mcp9808_registers address, const uint8_t value);
	int m_register_read16(const enum mcp9808_registers address, uint16_t * const value);
	int m_register_write16(const enum mcp9808_registers address, const uint16_t value);
};

#endif
