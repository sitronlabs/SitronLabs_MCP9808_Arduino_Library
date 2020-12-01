/* Self header */
#include "mcp9808.h"

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* Config */
#define CONFIG_MCP9808_I2CADDR_DEFAULT 0x18

/*!
 *    @brief  Instantiates a new MCP9808 class
 */
mcp9808::mcp9808() {
	m_i2c_address = CONFIG_MCP9808_I2CADDR_DEFAULT;
	m_i2c_library = NULL;
}

/*!
 *    @brief  Setups the HW with default address
 *    @return True if initialization was successful, otherwise false.
 */
void mcp9808::setup() {
	m_i2c_address = CONFIG_MCP9808_I2CADDR_DEFAULT;
	m_i2c_library = &Wire;
}

/**
 *
 * @param[in] i2c_library A pointer to an i2c library.
 */
void mcp9808::setup(TwoWire * const i2c_library) {
	m_i2c_library = i2c_library;
	m_i2c_address = CONFIG_MCP9808_I2CADDR_DEFAULT;
}

/*!
 *    @brief  Setups the HW
 *    @param  addr
 *    @return True if initialization was successful, otherwise false.
 */
void mcp9808::setup(const uint8_t i2c_address) {
	m_i2c_address = i2c_address;
	m_i2c_library = &Wire;
}

/*!
 *    @brief  Setups the HW
 *    @param  addr
 *    @param  *theWire
 *    @return True if initialization was successful, otherwise false.
 */
void mcp9808::setup(const uint8_t i2c_address, TwoWire * const i2c_library) {
	m_i2c_address = i2c_address;
	m_i2c_library = i2c_library;
}

/*!
 *    @brief  init function
 *    @return True if initialization was successful, otherwise false.
 */
bool mcp9808::detect(void) {
	int res;

	/* Ensure i2c library has been initialized */
	m_i2c_library->begin();

	/* Ensure manufacturer id matches */
	uint16_t reg_manufacturer_id;
	res = m_register_read16(MCP9808_REG_MANUF_ID, &reg_manufacturer_id);
	if (res < 0 || reg_manufacturer_id != 0x0054) return false;

	/* Ensure device id matches */
	uint16_t reg_device_id;
	res = m_register_read16(MCP9808_REG_DEVICE_ID, &reg_device_id);
	if (res < 0 || reg_device_id != 0x0400) return false;

	/* Return success */
	return true;
}

/**
 *
 * @return
 */
int mcp9808::resolution_get(enum mcp9808_resolution * const resolution) {
	int res;

	/* Read register */
	uint8_t reg_resolution;
	res = m_register_read8(MCP9808_REG_RESOLUTION, &reg_resolution);
	if (res < 0) {
		return -EIO;
	}
	*resolution = (enum mcp9808_resolution) (reg_resolution & 0x03);

	/* Return success */
	return 0;
}

/**
 *
 * @param value
 */
int mcp9808::resolution_set(const enum mcp9808_resolution resolution) {
	int res;

	/* Write register */
	uint8_t reg_resolution = resolution & 0x03;
	res = m_register_write8(MCP9808_REG_RESOLUTION, reg_resolution);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 * Reads the 16-bit temperature register and returns the temperature in degree Celsius as a float.
 * @param[out] temperature
 * @return 0 in case of success or a negative error code otherwise.
 * @see Section 5.1.3 of the datasheet.
 */
int mcp9808::temperature_read_celsius(float * const temperature) {
	int res;

	/* Read temperature register */
	uint16_t reg_temprature;
	res = m_register_read16(MCP9808_REG_TEMPERATURE, &reg_temprature);
	if (res < 0) {
		return -EIO;
	}

	/* Strip status bits */
	*temperature = 0x0FFF & reg_temprature;

	/* Shift the 4 decimal bits */
	*temperature /= 16.0;

	/* Consider the sign bit */
	if (reg_temprature & 0x1000) *temperature = 256 - *temperature;

	/* Return success */
	return 0;
}

/**
 *
 * @param[in] hysteresis
 * @return
 */
int mcp9808::alert_hysteresis_set(const enum mcp9808_alert_hysteresis hysteresis) {
	int res;

	/* Ensure parameter is valid */
	if (hysteresis != MCP9808_ALERT_HYSTERESIS_0_0 && hysteresis != MCP9808_ALERT_HYSTERESIS_1_5 && hysteresis != MCP9808_ALERT_HYSTERESIS_3_0 && hysteresis != MCP9808_ALERT_HYSTERESIS_6_0) {
		return -EINVAL;
	}

	/* Read register */
	uint16_t reg_config;
	res = m_register_read16(MCP9808_REG_CONFIG, &reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Modify register */
	reg_config &= 0xF9FF;
	reg_config |= (hysteresis << 9);

	/* Write register */
	res = m_register_write16(MCP9808_REG_CONFIG, reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @see Datasheet section 5.1.2.
 * @param[in] temperature
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::alert_threshold_lower_set(const float temperature) {
	int res;

	/* Prepare register */
	uint16_t reg_alert;
	if (temperature < 0) {
		reg_alert = -temperature * 16;
		reg_alert &= 0x0FFC;
		reg_alert |= 0x1000;
	} else {
		reg_alert = temperature * 16;
		reg_alert &= 0x0FFC;
	}

	/* Write register */
	res = m_register_write16(MCP9808_REG_ALERT_LOWER, reg_alert);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @see Datasheet section 5.1.2.
 * @param[in] temperature
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::alert_threshold_upper_set(const float temperature) {
	int res;

	/* Prepare register */
	uint16_t reg_alert;
	if (temperature < 0) {
		reg_alert = -temperature * 16;
		reg_alert &= 0x0FFC;
		reg_alert |= 0x1000;
	} else {
		reg_alert = temperature * 16;
		reg_alert &= 0x0FFC;
	}

	/* Write register */
	res = m_register_write16(MCP9808_REG_ALERT_UPPER, reg_alert);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @see Datasheet section 5.1.2.
 * @param[in] temperature
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::alert_threshold_critical_set(const float temperature) {
	int res;

	/* Prepare register */
	uint16_t reg_alert;
	if (temperature < 0) {
		reg_alert = -temperature * 16;
		reg_alert &= 0x0FFC;
		reg_alert |= 0x1000;
	} else {
		reg_alert = temperature * 16;
		reg_alert &= 0x0FFC;
	}

	/* Write register */
	res = m_register_write16(MCP9808_REG_ALERT_CRITICAL, reg_alert);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @return
 */
int mcp9808::alert_output_set(const boolean enabed, const enum mcp9808_alert_output_mode mode, const enum mcp9808_alert_output_polarity polarity) {
	int res;

	/* Read register */
	uint16_t reg_config;
	res = m_register_read16(MCP9808_REG_CONFIG, &reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Modify register*/
	reg_config &= ~0x000B;
	if (enabed) {
		reg_config |= (1 << 3);
		reg_config |= (polarity << 1);
		reg_config |= (mode << 0);
	}

	/* Write register */
	res = m_register_write16(MCP9808_REG_CONFIG, reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @return
 */
int mcp9808::interrupt_clear(void) {
	int res;

	/* Read register */
	uint16_t reg_config;
	res = m_register_read16(MCP9808_REG_CONFIG, &reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Modify register */
	reg_config &= ~0x0020;

	/* Write register */
	res = m_register_write16(MCP9808_REG_CONFIG, reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 * Transitions the sensor into shutdown mode where no measurement is being performed.
 * @param[in] shutdown True will transition the sensor into shutdown, and false will resume normal operation.
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::shutdown_set(const boolean shutdown) {
	int res;

	/* Read register */
	uint16_t reg_config;
	res = m_register_read16(MCP9808_REG_CONFIG, &reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Modify register */
	if (shutdown == true) reg_config |= 0x0100;
	else reg_config &= ~0x0100;

	/* Write register */
	res = m_register_write16(MCP9808_REG_CONFIG, reg_config);
	if (res < 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @param[in] address
 * @param[out] value
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::m_register_read8(const enum mcp9808_registers address, uint8_t * const value) {
	int res;

	/* Ensure library has been configured */
	if (m_i2c_library == NULL) {
		return -EINVAL;
	}

	/* Send register address */
	m_i2c_library->beginTransmission(m_i2c_address);
	m_i2c_library->write((uint8_t) address);
	res = m_i2c_library->endTransmission();
	if (res != 0) {
		return -EIO;
	}

	/* Read register value */
	m_i2c_library->requestFrom((uint8_t) m_i2c_address, (uint8_t) 1);
	*value = m_i2c_library->read();

	/* Return success */
	return 0;
}

/**
 *
 * @param[in] address
 * @param[in] value
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::m_register_write8(const enum mcp9808_registers address, const uint8_t value) {
	int res;

	/* Ensure library has been configured */
	if (m_i2c_library == NULL) {
		return -EINVAL;
	}

	/* Send register address and value */
	m_i2c_library->beginTransmission(m_i2c_address);
	m_i2c_library->write((uint8_t) address);
	m_i2c_library->write(value);
	res = m_i2c_library->endTransmission();
	if (res != 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @param[in] address
 * @param[out] value
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::m_register_read16(const enum mcp9808_registers address, uint16_t * const value) {
	int res;

	/* Ensure library has been configured */
	if (m_i2c_library == NULL) {
		return -EINVAL;
	}

	/* Send register address */
	m_i2c_library->beginTransmission(m_i2c_address);
	m_i2c_library->write((uint8_t) address);
	res = m_i2c_library->endTransmission();
	if (res != 0) {
		return -EIO;
	}

	/* Read register value */
	m_i2c_library->requestFrom((uint8_t) m_i2c_address, (uint8_t) 2);
	*value = m_i2c_library->read();
	*value <<= 8;
	*value |= m_i2c_library->read();

	/* Return success */
	return 0;
}

/**
 *
 * @param[in] address
 * @param[in] value
 * @return 0 in case of success or a negative error code otherwise.
 */
int mcp9808::m_register_write16(const enum mcp9808_registers address, const uint16_t value) {
	int res;

	/* Ensure library has been configured */
	if (m_i2c_library == NULL) {
		return -EINVAL;
	}

	/* Send register address and value */
	m_i2c_library->beginTransmission(m_i2c_address);
	m_i2c_library->write((uint8_t) address);
	m_i2c_library->write(value >> 8);
	m_i2c_library->write(value & 0xFF);
	res = m_i2c_library->endTransmission();
	if (res != 0) {
		return -EIO;
	}

	/* Return success */
	return 0;
}
