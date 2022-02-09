Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-bme280/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/bme280/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_BME280/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_BME280/actions/
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

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


Installing to a connected CircuitPython Device
==============================================
Some devices, eg. the QT-PY, are very limited in memory. The BME280 library contains
two variants - basic and advanced - which give different levels of functionality.

Installing the BME280 library could have the following outcomes:

    * It installs successfully and your code runs successfully. Woo-hoo! Continue with
      your amazing project.
    * It installs successfully and your code fails to run with a memory allocation
      error. Try one of the following:

        * If your ``code.py`` is large, especially if it has lots of comments, you
          can shrink it into a ``.mpy`` file instead. See the Adafruit
          `Learn Guide <https://learn.adafruit.com/Memory-saving-tips-for-CircuitPython/non-volatile-not-enough-disk-space>`_
          on shrinking your code.
        * Only use the basic BME280 implementation, and remove the following file:
          ``<CIRCUITPY>/lib/adafruit_bme280/advanced.mpy`` where <CIRCUITPY> is the
          mounted location of your device. Make sure that your code only uses the basic
          implementation.


Usage Example
=============

.. code-block:: python3

    import board
    import time
    from adafruit_bme280 import basic as adafruit_bme280

    # Create sensor object, using the board's default I2C bus.
    i2c = board.I2C()   # uses board.SCL and board.SDA
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    # change this to match the location's pressure (hPa) at sea level
    bme280.sea_level_pressure = 1013.25

    while True:
        print("\nTemperature: %0.1f C" % bme280.temperature)
        print("Humidity: %0.1f %%" % bme280.relative_humidity)
        print("Pressure: %0.1f hPa" % bme280.pressure)
        print("Altitude = %0.2f meters" % bme280.altitude)
        time.sleep(2)

Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/bme280/en/latest/>`_.

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_BME280/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
