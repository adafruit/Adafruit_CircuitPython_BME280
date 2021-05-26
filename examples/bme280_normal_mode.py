# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Example showing how the BME280 library can be used to set the various
parameters supported by the sensor.
Refer to the BME280 datasheet to understand what these parameters do
"""
import time
import board
from adafruit_bme280 import advanced

# Create sensor object, using the board's default I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA
bme280 = advanced.Adafruit_BME280_I2C(i2c)

# OR create sensor object, using the board's default SPI bus.
# SPI setup
# from digitalio import DigitalInOut
# spi = board.SPI()
# bme_cs = digitalio.DigitalInOut(board.D10)
# bme280 = advanced.Adafruit_BME280_SPI(spi, bme_cs)

# Change this to match the location's pressure (hPa) at sea level
bme280.sea_level_pressure = 1013.25
bme280.mode = advanced.MODE_NORMAL
bme280.standby_period = advanced.STANDBY_TC_500
bme280.iir_filter = advanced.IIR_FILTER_X16
bme280.overscan_pressure = advanced.OVERSCAN_X16
bme280.overscan_humidity = advanced.OVERSCAN_X1
bme280.overscan_temperature = advanced.OVERSCAN_X2
# The sensor will need a moment to gather initial readings
time.sleep(1)

while True:
    print("\nTemperature: %0.1f C" % bme280.temperature)
    print("Humidity: %0.1f %%" % bme280.relative_humidity)
    print("Pressure: %0.1f hPa" % bme280.pressure)
    print("Altitude = %0.2f meters" % bme280.altitude)
    time.sleep(2)
