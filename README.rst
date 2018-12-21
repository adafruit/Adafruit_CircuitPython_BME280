Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-bme280/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/bme280/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.com/adafruit/Adafruit_CircuitPython_BME280.svg?branch=master
    :target: https://travis-ci.com/adafruit/Adafruit_CircuitPython_BME280
    :alt: Build Status

I2C and SPI driver for the Bosch BME280 Temperature, Humidity, and Barometric Pressure sensor

Installation and Dependencies
=============================

This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure that the driver and all dependencies are available on the
CircuitPython filesystem.  This can be most easily achieved by downloading and
installing the latest
`Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_
on your device.

Installing from PyPI
--------------------

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-bme280/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-bme280

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-bme280

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-bme280

Usage Example
=============

.. code-block:: python

    import board
    import digitalio
    import busio
    import time
    import adafruit_bme280

    # Create library object using our Bus I2C port
    i2c = busio.I2C(board.SCL, board.SDA)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    # OR create library object using our Bus SPI port
    #spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
    #bme_cs = digitalio.DigitalInOut(board.D10)
    #bme280 = adafruit_bme280.Adafruit_BME280_SPI(spi, bme_cs)

    # change this to match the location's pressure (hPa) at sea level
    bme280.sea_level_pressure = 1013.25

    while True:
        print("\nTemperature: %0.1f C" % bme280.temperature)
        print("Humidity: %0.1f %%" % bme280.humidity)
        print("Pressure: %0.1f hPa" % bme280.pressure)
        print("Altitude = %0.2f meters" % bme280.altitude)
        time.sleep(2)

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_BME280/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building Locally
================

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip3 install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-veml6070 --library_location .

Sphinx Documentation
--------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip3 install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
