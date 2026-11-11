# SPDX-FileCopyrightText: 2026 Shounak Joshi for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""Tests that the driver imports on a board without Blinka's platform library."""

import importlib
import importlib.machinery
import sys

import pytest

import adafruit_bme280.advanced
import adafruit_bme280.basic
import adafruit_bme280.protocol

MODULES = ("adafruit_bme280.protocol", "adafruit_bme280.basic", "adafruit_bme280.advanced")
PLATFORM_MODULES = {"busio", "digitalio"}


class BlinkaPlatformLibraryMissing:
    """Raises the way Blinka does when its platform library is not installed."""

    @staticmethod
    def find_spec(fullname, path=None, target=None):
        """Refuses to find the platform modules, the way Blinka without its library does."""
        if fullname in PLATFORM_MODULES:
            raise RuntimeError(
                "The platform library 'adafruit_blinka_raspberry_pi_pwm' was not found. "
                "To install, try typing: ... 'Adafruit-Blinka-Raspberry-Pi-PWM>=1.0.2'"
            ) from ImportError("libgpiod Python bindings not found, please install and try again!")


class OtherFailure:
    """Fails with an error that has nothing to do with a missing library."""

    @staticmethod
    def find_spec(fullname, path=None, target=None):
        """Fails to find the platform modules."""
        if fullname in PLATFORM_MODULES:
            raise RuntimeError("the board is on fire")


def reimport(monkeypatch, finder):
    """Imports the driver modules again, with finder in front of the import machinery."""
    for name in (*MODULES, *PLATFORM_MODULES):
        monkeypatch.delitem(sys.modules, name, raising=False)
    monkeypatch.setattr(sys, "meta_path", [finder, *sys.meta_path])
    for name in MODULES:
        importlib.import_module(name)


def test_imports_without_blinka_platform_library(monkeypatch):
    """A board with only an I2C sensor wired up imports the driver."""
    reimport(monkeypatch, BlinkaPlatformLibraryMissing())


def test_other_runtime_errors_are_not_swallowed(monkeypatch):
    """A RuntimeError that is not Blinka's missing platform library still raises."""
    with pytest.raises(RuntimeError, match="the board is on fire"):
        reimport(monkeypatch, OtherFailure())
