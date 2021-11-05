# SPDX-FileCopyrightText: 2017 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_bme280.advanced`
=========================================================================================

CircuitPython driver from BME280 Temperature, Humidity and Barometric
Pressure sensor

* Author(s): ladyada, Jose David M.

Implementation Notes
--------------------

**Hardware:**

* `Adafruit BME280 Temperature, Humidity and Barometric Pressure sensor
  <https://www.adafruit.com/product/2652>`_ (Product ID: 2652)


**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

"""
from micropython import const
from adafruit_bme280.basic import Adafruit_BME280

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BME280.git"

#    I2C ADDRESS/BITS/SETTINGS
#    -----------------------------------------------------------------------
_BME280_ADDRESS = const(0x77)
_BME280_CHIPID = const(0x60)

_BME280_REGISTER_CHIPID = const(0xD0)
_BME280_REGISTER_DIG_T1 = const(0x88)
_BME280_REGISTER_DIG_H1 = const(0xA1)
_BME280_REGISTER_DIG_H2 = const(0xE1)
_BME280_REGISTER_DIG_H3 = const(0xE3)
_BME280_REGISTER_DIG_H4 = const(0xE4)
_BME280_REGISTER_DIG_H5 = const(0xE5)
_BME280_REGISTER_DIG_H6 = const(0xE7)

_BME280_REGISTER_SOFTRESET = const(0xE0)
_BME280_REGISTER_CTRL_HUM = const(0xF2)
_BME280_REGISTER_STATUS = const(0xF3)
_BME280_REGISTER_CTRL_MEAS = const(0xF4)
_BME280_REGISTER_CONFIG = const(0xF5)
_BME280_REGISTER_PRESSUREDATA = const(0xF7)
_BME280_REGISTER_TEMPDATA = const(0xFA)
_BME280_REGISTER_HUMIDDATA = const(0xFD)

_BME280_HUMIDITY_MIN = const(0)
_BME280_HUMIDITY_MAX = const(100)

"""iir_filter values"""
IIR_FILTER_DISABLE = const(0)
IIR_FILTER_X2 = const(0x01)
IIR_FILTER_X4 = const(0x02)
IIR_FILTER_X8 = const(0x03)
IIR_FILTER_X16 = const(0x04)

_BME280_IIR_FILTERS = (
    IIR_FILTER_DISABLE,
    IIR_FILTER_X2,
    IIR_FILTER_X4,
    IIR_FILTER_X8,
    IIR_FILTER_X16,
)

"""overscan values for temperature, pressure, and humidity"""
OVERSCAN_DISABLE = const(0x00)
OVERSCAN_X1 = const(0x01)
OVERSCAN_X2 = const(0x02)
OVERSCAN_X4 = const(0x03)
OVERSCAN_X8 = const(0x04)
OVERSCAN_X16 = const(0x05)

_BME280_OVERSCANS = {
    OVERSCAN_DISABLE: 0,
    OVERSCAN_X1: 1,
    OVERSCAN_X2: 2,
    OVERSCAN_X4: 4,
    OVERSCAN_X8: 8,
    OVERSCAN_X16: 16,
}

"""mode values"""
MODE_SLEEP = const(0x00)
MODE_FORCE = const(0x01)
MODE_NORMAL = const(0x03)

_BME280_MODES = (MODE_SLEEP, MODE_FORCE, MODE_NORMAL)
"""
standby timeconstant values
TC_X[_Y] where X=milliseconds and Y=tenths of a millisecond
"""
STANDBY_TC_0_5 = const(0x00)  # 0.5ms
STANDBY_TC_10 = const(0x06)  # 10ms
STANDBY_TC_20 = const(0x07)  # 20ms
STANDBY_TC_62_5 = const(0x01)  # 62.5ms
STANDBY_TC_125 = const(0x02)  # 125ms
STANDBY_TC_250 = const(0x03)  # 250ms
STANDBY_TC_500 = const(0x04)  # 500ms
STANDBY_TC_1000 = const(0x05)  # 1000ms

_BME280_STANDBY_TCS = (
    STANDBY_TC_0_5,
    STANDBY_TC_10,
    STANDBY_TC_20,
    STANDBY_TC_62_5,
    STANDBY_TC_125,
    STANDBY_TC_250,
    STANDBY_TC_500,
    STANDBY_TC_1000,
)

# pylint: disable=abstract-method
class Adafruit_BME280_Advanced(Adafruit_BME280):
    """Driver from BME280 Temperature, Humidity and Barometric Pressure sensor

    .. note::
        The operational range of the BMP280 is 300-1100 hPa.
        Pressure measurements outside this range may not be as accurate.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self):
        """Check the BME280 was found, read the coefficients and enable the sensor"""
        self._overscan_humidity = OVERSCAN_X1
        self._overscan_temperature = OVERSCAN_X1
        self._overscan_pressure = OVERSCAN_X16
        self._mode = MODE_SLEEP
        self._t_standby = STANDBY_TC_125
        super().__init__()

    @property
    def standby_period(self):
        """
        Control the inactive period when in Normal mode
        Allowed standby periods are the constants STANDBY_TC_*
        """
        return self._t_standby

    @standby_period.setter
    def standby_period(self, value):
        if not value in _BME280_STANDBY_TCS:
            raise ValueError("Standby Period '%s' not supported" % (value))
        if self._t_standby == value:
            return
        self._t_standby = value
        self._write_config()

    @property
    def overscan_humidity(self):
        """
        Humidity Oversampling
        Allowed values are the constants OVERSCAN_*
        """
        return self._overscan_humidity

    @overscan_humidity.setter
    def overscan_humidity(self, value):
        if not value in _BME280_OVERSCANS:
            raise ValueError("Overscan value '%s' not supported" % (value))
        self._overscan_humidity = value
        self._write_ctrl_meas()

    @property
    def overscan_temperature(self):
        """
        Temperature Oversampling
        Allowed values are the constants OVERSCAN_*
        """
        return self._overscan_temperature

    @overscan_temperature.setter
    def overscan_temperature(self, value):
        if not value in _BME280_OVERSCANS:
            raise ValueError("Overscan value '%s' not supported" % (value))
        self._overscan_temperature = value
        self._write_ctrl_meas()

    @property
    def overscan_pressure(self):
        """
        Pressure Oversampling
        Allowed values are the constants OVERSCAN_*
        """
        return self._overscan_pressure

    @overscan_pressure.setter
    def overscan_pressure(self, value):
        if not value in _BME280_OVERSCANS:
            raise ValueError("Overscan value '%s' not supported" % (value))
        self._overscan_pressure = value
        self._write_ctrl_meas()

    @property
    def iir_filter(self):
        """
        Controls the time constant of the IIR filter
        Allowed values are the constants IIR_FILTER_*
        """
        return self._iir_filter

    @iir_filter.setter
    def iir_filter(self, value):
        if not value in _BME280_IIR_FILTERS:
            raise ValueError("IIR Filter '%s' not supported" % (value))
        self._iir_filter = value
        self._write_config()

    @property
    def _config(self):
        """Value to be written to the device's config register """
        config = 0
        if self.mode == MODE_NORMAL:
            config += self._t_standby << 5
        if self._iir_filter:
            config += self._iir_filter << 2
        return config

    @property
    def _ctrl_meas(self):
        """Value to be written to the device's ctrl_meas register """
        ctrl_meas = self.overscan_temperature << 5
        ctrl_meas += self.overscan_pressure << 2
        ctrl_meas += self.mode
        return ctrl_meas

    @property
    def measurement_time_typical(self):
        """Typical time in milliseconds required to complete a measurement in normal mode"""
        meas_time_ms = 1.0
        if self.overscan_temperature != OVERSCAN_DISABLE:
            meas_time_ms += 2 * _BME280_OVERSCANS.get(self.overscan_temperature)
        if self.overscan_pressure != OVERSCAN_DISABLE:
            meas_time_ms += 2 * _BME280_OVERSCANS.get(self.overscan_pressure) + 0.5
        if self.overscan_humidity != OVERSCAN_DISABLE:
            meas_time_ms += 2 * _BME280_OVERSCANS.get(self.overscan_humidity) + 0.5
        return meas_time_ms

    @property
    def measurement_time_max(self):
        """Maximum time in milliseconds required to complete a measurement in normal mode"""
        meas_time_ms = 1.25
        if self.overscan_temperature != OVERSCAN_DISABLE:
            meas_time_ms += 2.3 * _BME280_OVERSCANS.get(self.overscan_temperature)
        if self.overscan_pressure != OVERSCAN_DISABLE:
            meas_time_ms += 2.3 * _BME280_OVERSCANS.get(self.overscan_pressure) + 0.575
        if self.overscan_humidity != OVERSCAN_DISABLE:
            meas_time_ms += 2.3 * _BME280_OVERSCANS.get(self.overscan_humidity) + 0.575
        return meas_time_ms


class Adafruit_BME280_I2C(Adafruit_BME280_Advanced):
    """Driver for BME280 connected over I2C

    :param ~busio.I2C i2c: The I2C bus the BME280 is connected to.
    :param int address: I2C device address. Defaults to :const:`0x77`.
                        but another address can be passed in as an argument

    .. note::
        The operational range of the BMP280 is 300-1100 hPa.
        Pressure measurements outside this range may not be as accurate.

    **Quickstart: Importing and using the BME280**

        Here is an example of using the :class:`Adafruit_BME280_I2C`.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import adafruit_bme280.advanced as adafruit_bme280

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()   # uses board.SCL and board.SDA
            bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

        You need to setup the pressure at sea level

        .. code-block:: python

            bme280.sea_level_pressure = 1013.25

        Now you have access to the :attr:`temperature`, :attr:`relative_humidity`
        :attr:`pressure` and :attr:`altitude` attributes

        .. code-block:: python

            temperature = bme280.temperature
            relative_humidity = bme280.relative_humidity
            pressure = bme280.pressure
            altitude = bme280.altitude

    """

    def __init__(self, i2c, address=_BME280_ADDRESS):
        from adafruit_bus_device.i2c_device import (  # pylint: disable=import-outside-toplevel
            i2c_device,
        )

        self._i2c = i2c_device.I2CDevice(i2c, address)
        super().__init__()

    def _read_register(self, register, length):
        with self._i2c as i2c:
            i2c.write(bytes([register & 0xFF]))
            result = bytearray(length)
            i2c.readinto(result)
            # print("$%02X => %s" % (register, [hex(i) for i in result]))
            return result

    def _write_register_byte(self, register, value):
        with self._i2c as i2c:
            i2c.write(bytes([register & 0xFF, value & 0xFF]))
            # print("$%02X <= 0x%02X" % (register, value))


class Adafruit_BME280_SPI(Adafruit_BME280_Advanced):
    """Driver for BME280 connected over SPI

    :param ~busio.SPI spi: SPI device
    :param ~digitalio.DigitalInOut cs: Chip Select
    :param int baudrate: Clock rate, default is 100000. Can be changed with :meth:`baudrate`

    .. note::
        The operational range of the BMP280 is 300-1100 hPa.
        Pressure measurements outside this range may not be as accurate.

    **Quickstart: Importing and using the BME280**

        Here is an example of using the :class:`Adafruit_BME280_SPI` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from digitalio import DigitalInOut
            import adafruit_bme280.advanced as adafruit_bme280

        Once this is done you can define your `board.SPI` object and define your sensor object

        .. code-block:: python

            cs = digitalio.DigitalInOut(board.D10)
            spi = board.SPI()
            bme280 = adafruit_bme280.Adafruit_BME280_SPI(spi, cs)

        You need to setup the pressure at sea level

        .. code-block:: python

            bme280.sea_level_pressure = 1013.25

        Now you have access to the :attr:`temperature`, :attr:`relative_humidity`
        :attr:`pressure` and :attr:`altitude` attributes

        .. code-block:: python

            temperature = bme280.temperature
            relative_humidity = bme280.relative_humidity
            pressure = bme280.pressure
            altitude = bme280.altitude

    """

    def __init__(self, spi, cs, baudrate=100000):
        from adafruit_bus_device.spi_device import (  # pylint: disable=import-outside-toplevel
            spi_device,
        )

        self._spi = spi_device.SPIDevice(spi, cs, baudrate=baudrate)
        super().__init__()

    def _read_register(self, register, length):
        register = (register | 0x80) & 0xFF  # Read single, bit 7 high.
        with self._spi as spi:
            spi.write(bytearray([register]))  # pylint: disable=no-member
            result = bytearray(length)
            spi.readinto(result)  # pylint: disable=no-member
            # print("$%02X => %s" % (register, [hex(i) for i in result]))
            return result

    def _write_register_byte(self, register, value):
        register &= 0x7F  # Write, bit 7 low.
        with self._spi as spi:
            spi.write(bytes([register, value & 0xFF]))  # pylint: disable=no-member
