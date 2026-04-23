# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time

import board
import busio

from adafruit_bme280 import basic as adafruit_bme280

# Create sensor object, using the board's default I2C bus.
i2c = busio.I2C(board.GP1, board.GP0)  # SCL, SDA
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# OR create sensor object, using the board's default SPI bus.
# spi = busio.SPI(board.GP2, MISO=board.GP0, MOSI=board.GP3)
# bme_cs = digitalio.DigitalInOut(board.GP1)
# bme280 = adafruit_bme280.Adafruit_BME280_SPI(spi, bme_cs)

# change this to match the location's pressure (hPa) at sea level
bme280.sea_level_pressure = 1013.25

while True:
    print(f"\nTemperature: {bme280.temperature:0.1f} C")
    print(f"Humidity: {bme280.relative_humidity:0.1f} %")
    print(f"Pressure: {bme280.pressure:0.1f} hPa")
    print(f"Altitude = {bme280.altitude:0.2f} meters")
    time.sleep(2)
