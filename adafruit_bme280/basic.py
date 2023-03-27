# SPDX-FileCopyrightText: 2017 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_bme280.basic`
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
import math
import struct
from time import sleep

from micropython import const
from adafruit_bme280.protocol import I2C_Impl, SPI_Impl

try:
    import typing  # pylint: disable=unused-import
    from busio import I2C, SPI
    from digitalio import DigitalInOut
except ImportError:
    pass

__version__ = "2.6.4"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BME280.git"

#    I2C ADDRESS/BITS/SETTINGS
#    -----------------------------------------------------------------------

"""General Information"""
_BME280_ADDRESS = const(0x77)
_BME280_CHIPID = const(0x60)
_BME280_REGISTER_CHIPID = const(0xD0)
"""overscan values for temperature, pressure, and humidity"""
OVERSCAN_X1 = const(0x01)
OVERSCAN_X16 = const(0x05)
"""mode values"""
_BME280_MODES = (0x00, 0x01, 0x03)
"""iir_filter values"""
IIR_FILTER_DISABLE = const(0)
"""
standby timeconstant values
TC_X[_Y] where X=milliseconds and Y=tenths of a millisecond
"""
STANDBY_TC_125 = const(0x02)  # 125ms
"""mode values"""
MODE_SLEEP = const(0x00)
MODE_FORCE = const(0x01)
MODE_NORMAL = const(0x03)
"""Other Registers"""
_BME280_REGISTER_SOFTRESET = const(0xE0)
_BME280_REGISTER_CTRL_HUM = const(0xF2)
_BME280_REGISTER_STATUS = const(0xF3)
_BME280_REGISTER_CTRL_MEAS = const(0xF4)
_BME280_REGISTER_CONFIG = const(0xF5)
_BME280_REGISTER_TEMPDATA = const(0xFA)
_BME280_REGISTER_HUMIDDATA = const(0xFD)


class Adafruit_BME280:
    """Driver from BME280 Temperature, Humidity and Barometric Pressure sensor

    .. note::
        The operational range of the BME280 is 300-1100 hPa.
        Pressure measurements outside this range may not be as accurate.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, bus_implementation: typing.Union[I2C_Impl, SPI_Impl]) -> None:
        """Check the BME280 was found, read the coefficients and enable the sensor"""
        # Check device ID.
        self._bus_implementation = bus_implementation
        chip_id = self._read_byte(_BME280_REGISTER_CHIPID)
        if _BME280_CHIPID != chip_id:
            raise RuntimeError("Failed to find BME280! Chip ID 0x%x" % chip_id)
        # Set some reasonable defaults.
        self._iir_filter = IIR_FILTER_DISABLE
        self.overscan_humidity = OVERSCAN_X1
        self.overscan_temperature = OVERSCAN_X1
        self.overscan_pressure = OVERSCAN_X16
        self._t_standby = STANDBY_TC_125
        self._mode = MODE_SLEEP
        self._reset()
        self._read_coefficients()
        self._write_ctrl_meas()
        self._write_config()
        self.sea_level_pressure = 1013.25
        """Pressure in hectoPascals at sea level. Used to calibrate `altitude`."""
        self._t_fine = None

    def _read_temperature(self) -> None:
        # perform one measurement
        if self.mode != MODE_NORMAL:
            self.mode = MODE_FORCE
            # Wait for conversion to complete
            while self._get_status() & 0x08:
                sleep(0.002)
        raw_temperature = (
            self._read24(_BME280_REGISTER_TEMPDATA) / 16
        )  # lowest 4 bits get dropped

        var1 = (
            raw_temperature / 16384.0 - self._temp_calib[0] / 1024.0
        ) * self._temp_calib[1]

        var2 = (
            (raw_temperature / 131072.0 - self._temp_calib[0] / 8192.0)
            * (raw_temperature / 131072.0 - self._temp_calib[0] / 8192.0)
        ) * self._temp_calib[2]

        self._t_fine = int(var1 + var2)

    def _reset(self) -> None:
        """Soft reset the sensor"""
        self._write_register_byte(_BME280_REGISTER_SOFTRESET, 0xB6)
        sleep(0.004)  # Datasheet says 2ms.  Using 4ms just to be safe

    def _write_ctrl_meas(self) -> None:
        """
        Write the values to the ctrl_meas and ctrl_hum registers in the device
        ctrl_meas sets the pressure and temperature data acquisition options
        ctrl_hum sets the humidity oversampling and must be written to first
        """
        self._write_register_byte(_BME280_REGISTER_CTRL_HUM, self.overscan_humidity)
        self._write_register_byte(_BME280_REGISTER_CTRL_MEAS, self._ctrl_meas)

    def _get_status(self) -> int:
        """Get the value from the status register in the device"""
        return self._read_byte(_BME280_REGISTER_STATUS)

    def _read_config(self) -> int:
        """Read the value from the config register in the device"""
        return self._read_byte(_BME280_REGISTER_CONFIG)

    def _write_config(self) -> None:
        """Write the value to the config register in the device"""
        normal_flag = False
        if self._mode == MODE_NORMAL:
            # Writes to the config register may be ignored while in Normal mode
            normal_flag = True
            self.mode = MODE_SLEEP  # So we switch to Sleep mode first
        self._write_register_byte(_BME280_REGISTER_CONFIG, self._config)
        if normal_flag:
            self.mode = MODE_NORMAL

    @property
    def mode(self) -> int:
        """
        Operation mode
        Allowed values are the constants MODE_*
        """
        return self._mode

    @mode.setter
    def mode(self, value: int) -> None:
        if not value in _BME280_MODES:
            raise ValueError("Mode '%s' not supported" % (value))
        self._mode = value
        self._write_ctrl_meas()

    @property
    def _config(self) -> int:
        """Value to be written to the device's config register"""
        config = 0
        if self.mode == 0x03:  # MODE_NORMAL
            config += self._t_standby << 5
        if self._iir_filter:
            config += self._iir_filter << 2
        return config

    @property
    def _ctrl_meas(self) -> int:
        """Value to be written to the device's ctrl_meas register"""
        ctrl_meas = self.overscan_temperature << 5
        ctrl_meas += self.overscan_pressure << 2
        ctrl_meas += self.mode
        return ctrl_meas

    @property
    def temperature(self) -> float:
        """The compensated temperature in degrees Celsius."""
        self._read_temperature()
        return self._t_fine / 5120.0

    @property
    def pressure(self) -> float:
        """
        The compensated pressure in hectoPascals.
        """
        self._read_temperature()

        # Algorithm from the BME280 driver
        # https://github.com/BoschSensortec/BME280_driver/blob/master/bme280.c
        adc = (
            self._read24(0xF7) / 16  # BME280_REGISTER_PRESSUREDATA
        )  # lowest 4 bits get dropped
        var1 = float(self._t_fine) / 2.0 - 64000.0
        var2 = var1 * var1 * self._pressure_calib[5] / 32768.0
        var2 = var2 + var1 * self._pressure_calib[4] * 2.0
        var2 = var2 / 4.0 + self._pressure_calib[3] * 65536.0
        var3 = self._pressure_calib[2] * var1 * var1 / 524288.0
        var1 = (var3 + self._pressure_calib[1] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._pressure_calib[0]
        if not var1:  # avoid exception caused by division by zero
            raise ArithmeticError(
                "Invalid result possibly related to error while reading the calibration registers"
            )
        pressure = 1048576.0 - adc
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = self._pressure_calib[8] * pressure * pressure / 2147483648.0
        var2 = pressure * self._pressure_calib[7] / 32768.0
        pressure = pressure + (var1 + var2 + self._pressure_calib[6]) / 16.0

        pressure /= 100
        return pressure

    @property
    def relative_humidity(self) -> float:
        """
        The relative humidity in RH %
        """
        return self.humidity

    @property
    def humidity(self) -> float:
        """
        The relative humidity in RH %
        """
        self._read_temperature()
        hum = self._read_register(0xFD, 2)  # BME280_REGISTER_HUMIDDATA
        adc = float(hum[0] << 8 | hum[1])

        # Algorithm from the BME280 driver
        # https://github.com/BoschSensortec/BME280_driver/blob/master/bme280.c
        var1 = float(self._t_fine) - 76800.0
        var2 = (
            self._humidity_calib[3] * 64.0 + (self._humidity_calib[4] / 16384.0) * var1
        )
        var3 = adc - var2
        var4 = self._humidity_calib[1] / 65536.0
        var5 = 1.0 + (self._humidity_calib[2] / 67108864.0) * var1
        var6 = 1.0 + (self._humidity_calib[5] / 67108864.0) * var1 * var5
        var6 = var3 * var4 * (var5 * var6)
        humidity = var6 * (1.0 - self._humidity_calib[0] * var6 / 524288.0)

        if humidity > 100:
            return 100
        if humidity < 0:
            return 0
        # else...
        return humidity

    @property
    def altitude(self) -> float:
        """The altitude based on current :attr:`pressure` versus the sea level pressure
        (``sea_level_pressure``) - which you must enter ahead of time)"""
        pressure = self.pressure  # in Si units for hPascal
        return 44330 * (1.0 - math.pow(pressure / self.sea_level_pressure, 0.1903))

    def _read_coefficients(self) -> None:
        """Read & save the calibration coefficients"""
        coeff = self._read_register(0x88, 24)  # BME280_REGISTER_DIG_T1
        coeff = list(struct.unpack("<HhhHhhhhhhhh", bytes(coeff)))
        coeff = [float(i) for i in coeff]
        self._temp_calib = coeff[:3]
        self._pressure_calib = coeff[3:]

        self._humidity_calib = [0] * 6
        self._humidity_calib[0] = self._read_byte(0xA1)  # BME280_REGISTER_DIG_H1
        coeff = self._read_register(0xE1, 7)  # BME280_REGISTER_DIG_H2
        coeff = list(struct.unpack("<hBbBbb", bytes(coeff)))
        self._humidity_calib[1] = float(coeff[0])
        self._humidity_calib[2] = float(coeff[1])
        self._humidity_calib[3] = float((coeff[2] << 4) | (coeff[3] & 0xF))
        self._humidity_calib[4] = float((coeff[4] << 4) | (coeff[3] >> 4))
        self._humidity_calib[5] = float(coeff[5])

    def _read_byte(self, register: int) -> int:
        """Read a byte register value and return it"""
        return self._read_register(register, 1)[0]

    def _read24(self, register: int) -> float:
        """Read an unsigned 24-bit value as a floating point and return it."""
        ret = 0.0
        for b in self._read_register(register, 3):
            ret *= 256.0
            ret += float(b & 0xFF)
        return ret

    def _read_register(self, register: int, length: int) -> bytearray:
        return self._bus_implementation.read_register(register, length)

    def _write_register_byte(self, register: int, value: int) -> None:
        self._bus_implementation.write_register_byte(register, value)


class Adafruit_BME280_I2C(Adafruit_BME280):

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
        from adafruit_bme280 import basic as adafruit_bme280

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

    def __init__(self, i2c: I2C, address: int = 0x77) -> None:  # BME280_ADDRESS
        super().__init__(I2C_Impl(i2c, address))


class Adafruit_BME280_SPI(Adafruit_BME280):

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
            from adafruit_bme280 import basic as adafruit_bme280

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

    def __init__(self, spi: SPI, cs: DigitalInOut, baudrate: int = 100000) -> None:
        super().__init__(SPI_Impl(spi, cs, baudrate))
