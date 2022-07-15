# SPDX-FileCopyrightText: 2017 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# We have a lot of attributes for this complex sensor.
# pylint: disable=too-many-instance-attributes

"""
`adafruit_bme680`
================================================================================

CircuitPython library for BME680 temperature, pressure and humidity sensor.


* Author(s): Limor Fried

Implementation Notes
--------------------

**Hardware:**

* `Adafruit BME680 Temp, Humidity, Pressure and Gas Sensor <https://www.adafruit.com/product/3660>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import struct
import time
import math
from micropython import const

try:
    # Used only for type annotations.

    import typing  # pylint: disable=unused-import

    from circuitpython_typing import ReadableBuffer
    from busio import I2C, SPI
    from digitalio import DigitalInOut

except ImportError:
    pass

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BME680.git"


#    I2C ADDRESS/BITS/SETTINGS
#    -----------------------------------------------------------------------
_BME680_CHIPID = const(0x61)

_BME680_REG_CHIPID = const(0xD0)
_BME68X_REG_VARIANT = const(0xF0)
_BME680_BME680_COEFF_ADDR1 = const(0x89)
_BME680_BME680_COEFF_ADDR2 = const(0xE1)
_BME680_BME680_RES_HEAT_0 = const(0x5A)
_BME680_BME680_GAS_WAIT_0 = const(0x64)

_BME680_REG_SOFTRESET = const(0xE0)
_BME680_REG_CTRL_GAS = const(0x71)
_BME680_REG_CTRL_HUM = const(0x72)
_BME680_REG_STATUS = const(0x73)
_BME680_REG_CTRL_MEAS = const(0x74)
_BME680_REG_CONFIG = const(0x75)

_BME680_REG_MEAS_STATUS = const(0x1D)
_BME680_REG_PDATA = const(0x1F)
_BME680_REG_TDATA = const(0x22)
_BME680_REG_HDATA = const(0x25)

_BME680_SAMPLERATES = (0, 1, 2, 4, 8, 16)
_BME680_FILTERSIZES = (0, 1, 3, 7, 15, 31, 63, 127)

_BME680_RUNGAS = const(0x10)

_LOOKUP_TABLE_1 = (
    2147483647.0,
    2147483647.0,
    2147483647.0,
    2147483647.0,
    2147483647.0,
    2126008810.0,
    2147483647.0,
    2130303777.0,
    2147483647.0,
    2147483647.0,
    2143188679.0,
    2136746228.0,
    2147483647.0,
    2126008810.0,
    2147483647.0,
    2147483647.0,
)

_LOOKUP_TABLE_2 = (
    4096000000.0,
    2048000000.0,
    1024000000.0,
    512000000.0,
    255744255.0,
    127110228.0,
    64000000.0,
    32258064.0,
    16016016.0,
    8000000.0,
    4000000.0,
    2000000.0,
    1000000.0,
    500000.0,
    250000.0,
    125000.0,
)


def _read24(arr: ReadableBuffer) -> float:
    """Parse an unsigned 24-bit value as a floating point and return it."""
    ret = 0.0
    # print([hex(i) for i in arr])
    for b in arr:
        ret *= 256.0
        ret += float(b & 0xFF)
    return ret


class Adafruit_BME680:
    """Driver from BME680 air quality sensor

    :param int refresh_rate: Maximum number of readings per second. Faster property reads
      will be from the previous reading."""

    def __init__(self, *, refresh_rate: int = 10) -> None:
        """Check the BME680 was found, read the coefficients and enable the sensor for continuous
        reads."""
        self._write(_BME680_REG_SOFTRESET, [0xB6])
        time.sleep(0.005)

        # Check device ID.
        chip_id = self._read_byte(_BME680_REG_CHIPID)
        if chip_id != _BME680_CHIPID:
            raise RuntimeError("Failed to find BME680! Chip ID 0x%x" % chip_id)

        # Get variant
        self._chip_variant = self._read_byte(_BME68X_REG_VARIANT)

        self._read_calibration()

        # set up heater
        self._write(_BME680_BME680_RES_HEAT_0, [0x73])
        self._write(_BME680_BME680_GAS_WAIT_0, [0x65])

        self.sea_level_pressure = 1013.25
        """Pressure in hectoPascals at sea level. Used to calibrate :attr:`altitude`."""

        # Default oversampling and filter register values.
        self._pressure_oversample = 0b011
        self._temp_oversample = 0b100
        self._humidity_oversample = 0b010
        self._filter = 0b010

        self._adc_pres = None
        self._adc_temp = None
        self._adc_hum = None
        self._adc_gas = None
        self._gas_range = None
        self._t_fine = None

        self._last_reading = 0
        self._min_refresh_time = 1 / refresh_rate

    @property
    def pressure_oversample(self) -> int:
        """The oversampling for pressure sensor"""
        return _BME680_SAMPLERATES[self._pressure_oversample]

    @pressure_oversample.setter
    def pressure_oversample(self, sample_rate: int) -> None:
        if sample_rate in _BME680_SAMPLERATES:
            self._pressure_oversample = _BME680_SAMPLERATES.index(sample_rate)
        else:
            raise RuntimeError("Invalid oversample")

    @property
    def humidity_oversample(self) -> int:
        """The oversampling for humidity sensor"""
        return _BME680_SAMPLERATES[self._humidity_oversample]

    @humidity_oversample.setter
    def humidity_oversample(self, sample_rate: int) -> None:
        if sample_rate in _BME680_SAMPLERATES:
            self._humidity_oversample = _BME680_SAMPLERATES.index(sample_rate)
        else:
            raise RuntimeError("Invalid oversample")

    @property
    def temperature_oversample(self) -> int:
        """The oversampling for temperature sensor"""
        return _BME680_SAMPLERATES[self._temp_oversample]

    @temperature_oversample.setter
    def temperature_oversample(self, sample_rate: int) -> None:
        if sample_rate in _BME680_SAMPLERATES:
            self._temp_oversample = _BME680_SAMPLERATES.index(sample_rate)
        else:
            raise RuntimeError("Invalid oversample")

    @property
    def filter_size(self) -> int:
        """The filter size for the built in IIR filter"""
        return _BME680_FILTERSIZES[self._filter]

    @filter_size.setter
    def filter_size(self, size: int) -> None:
        if size in _BME680_FILTERSIZES:
            self._filter = _BME680_FILTERSIZES.index(size)
        else:
            raise RuntimeError("Invalid size")

    @property
    def temperature(self) -> float:
        """The compensated temperature in degrees Celsius."""
        self._perform_reading()
        calc_temp = ((self._t_fine * 5) + 128) / 256
        return calc_temp / 100

    @property
    def pressure(self) -> float:
        """The barometric pressure in hectoPascals"""
        self._perform_reading()
        var1 = (self._t_fine / 2) - 64000
        var2 = ((var1 / 4) * (var1 / 4)) / 2048
        var2 = (var2 * self._pressure_calibration[5]) / 4
        var2 = var2 + (var1 * self._pressure_calibration[4] * 2)
        var2 = (var2 / 4) + (self._pressure_calibration[3] * 65536)
        var1 = (
            (((var1 / 4) * (var1 / 4)) / 8192)
            * (self._pressure_calibration[2] * 32)
            / 8
        ) + ((self._pressure_calibration[1] * var1) / 2)
        var1 = var1 / 262144
        var1 = ((32768 + var1) * self._pressure_calibration[0]) / 32768
        calc_pres = 1048576 - self._adc_pres
        calc_pres = (calc_pres - (var2 / 4096)) * 3125
        calc_pres = (calc_pres / var1) * 2
        var1 = (
            self._pressure_calibration[8] * (((calc_pres / 8) * (calc_pres / 8)) / 8192)
        ) / 4096
        var2 = ((calc_pres / 4) * self._pressure_calibration[7]) / 8192
        var3 = (((calc_pres / 256) ** 3) * self._pressure_calibration[9]) / 131072
        calc_pres += (var1 + var2 + var3 + (self._pressure_calibration[6] * 128)) / 16
        return calc_pres / 100

    @property
    def relative_humidity(self) -> float:
        """The relative humidity in RH %"""
        return self.humidity

    @property
    def humidity(self) -> float:
        """The relative humidity in RH %"""
        self._perform_reading()
        temp_scaled = ((self._t_fine * 5) + 128) / 256
        var1 = (self._adc_hum - (self._humidity_calibration[0] * 16)) - (
            (temp_scaled * self._humidity_calibration[2]) / 200
        )
        var2 = (
            self._humidity_calibration[1]
            * (
                ((temp_scaled * self._humidity_calibration[3]) / 100)
                + (
                    (
                        (
                            temp_scaled
                            * ((temp_scaled * self._humidity_calibration[4]) / 100)
                        )
                        / 64
                    )
                    / 100
                )
                + 16384
            )
        ) / 1024
        var3 = var1 * var2
        var4 = self._humidity_calibration[5] * 128
        var4 = (var4 + ((temp_scaled * self._humidity_calibration[6]) / 100)) / 16
        var5 = ((var3 / 16384) * (var3 / 16384)) / 1024
        var6 = (var4 * var5) / 2
        calc_hum = (((var3 + var6) / 1024) * 1000) / 4096
        calc_hum /= 1000  # get back to RH

        calc_hum = min(calc_hum, 100)
        calc_hum = max(calc_hum, 0)
        return calc_hum

    @property
    def altitude(self) -> float:
        """The altitude based on current :attr:`pressure` vs the sea level pressure
        (:attr:`sea_level_pressure`) - which you must enter ahead of time)"""
        pressure = self.pressure  # in Si units for hPascal
        return 44330 * (1.0 - math.pow(pressure / self.sea_level_pressure, 0.1903))

    @property
    def gas(self) -> int:
        """The gas resistance in ohms"""
        self._perform_reading()
        if self._chip_variant == 0x01:
            # taken from https://github.com/BoschSensortec/BME68x-Sensor-API
            var1 = 262144 >> self._gas_range
            var2 = self._adc_gas - 512
            var2 *= 3
            var2 = 4096 + var2
            calc_gas_res = (1000 * var1) / var2
            calc_gas_res = calc_gas_res * 100
        else:
            var1 = (
                (1340 + (5 * self._sw_err)) * (_LOOKUP_TABLE_1[self._gas_range])
            ) / 65536
            var2 = ((self._adc_gas * 32768) - 16777216) + var1
            var3 = (_LOOKUP_TABLE_2[self._gas_range] * var1) / 512
            calc_gas_res = (var3 + (var2 / 2)) / var2
        return int(calc_gas_res)

    def _perform_reading(self) -> None:
        """Perform a single-shot reading from the sensor and fill internal data structure for
        calculations"""
        if time.time_ns() - self._last_reading < self._min_refresh_time:
            return

        # set filter
        self._write(_BME680_REG_CONFIG, [self._filter << 2])
        # turn on temp oversample & pressure oversample
        self._write(
            _BME680_REG_CTRL_MEAS,
            [(self._temp_oversample << 5) | (self._pressure_oversample << 2)],
        )
        # turn on humidity oversample
        self._write(_BME680_REG_CTRL_HUM, [self._humidity_oversample])
        # gas measurements enabled
        if self._chip_variant == 0x01:
            self._write(_BME680_REG_CTRL_GAS, [_BME680_RUNGAS << 1])
        else:
            self._write(_BME680_REG_CTRL_GAS, [_BME680_RUNGAS])
        ctrl = self._read_byte(_BME680_REG_CTRL_MEAS)
        ctrl = (ctrl & 0xFC) | 0x01  # enable single shot!
        self._write(_BME680_REG_CTRL_MEAS, [ctrl])
        new_data = False
        while not new_data:
            data = self._read(_BME680_REG_MEAS_STATUS, 17)
            new_data = data[0] & 0x80 != 0
            time.sleep(0.005)
        self._last_reading = time.time_ns()

        self._adc_pres = _read24(data[2:5]) / 16
        self._adc_temp = _read24(data[5:8]) / 16
        self._adc_hum = struct.unpack(">H", bytes(data[8:10]))[0]
        if self._chip_variant == 0x01:
            self._adc_gas = int(struct.unpack(">H", bytes(data[15:17]))[0] / 64)
            self._gas_range = data[16] & 0x0F
        else:
            self._adc_gas = int(struct.unpack(">H", bytes(data[13:15]))[0] / 64)
            self._gas_range = data[14] & 0x0F

        var1 = (self._adc_temp / 8) - (self._temp_calibration[0] * 2)
        var2 = (var1 * self._temp_calibration[1]) / 2048
        var3 = ((var1 / 2) * (var1 / 2)) / 4096
        var3 = (var3 * self._temp_calibration[2] * 16) / 16384
        self._t_fine = int(var2 + var3)

    def _read_calibration(self) -> None:
        """Read & save the calibration coefficients"""
        coeff = self._read(_BME680_BME680_COEFF_ADDR1, 25)
        coeff += self._read(_BME680_BME680_COEFF_ADDR2, 16)

        coeff = list(struct.unpack("<hbBHhbBhhbbHhhBBBHbbbBbHhbb", bytes(coeff[1:39])))
        # print("\n\n",coeff)
        coeff = [float(i) for i in coeff]
        self._temp_calibration = [coeff[x] for x in [23, 0, 1]]
        self._pressure_calibration = [
            coeff[x] for x in [3, 4, 5, 7, 8, 10, 9, 12, 13, 14]
        ]
        self._humidity_calibration = [coeff[x] for x in [17, 16, 18, 19, 20, 21, 22]]
        self._gas_calibration = [coeff[x] for x in [25, 24, 26]]

        # flip around H1 & H2
        self._humidity_calibration[1] *= 16
        self._humidity_calibration[1] += self._humidity_calibration[0] % 16
        self._humidity_calibration[0] /= 16

        self._heat_range = (self._read_byte(0x02) & 0x30) / 16
        self._heat_val = self._read_byte(0x00)
        self._sw_err = (self._read_byte(0x04) & 0xF0) / 16

    def _read_byte(self, register: int) -> int:
        """Read a byte register value and return it"""
        return self._read(register, 1)[0]

    def _read(self, register: int, length: int) -> bytearray:
        raise NotImplementedError()

    def _write(self, register: int, values: bytearray) -> None:
        raise NotImplementedError()


class Adafruit_BME680_I2C(Adafruit_BME680):
    """Driver for I2C connected BME680.

    :param ~busio.I2C i2c: The I2C bus the BME680 is connected to.
    :param int address: I2C device address. Defaults to :const:`0x77`
    :param bool debug: Print debug statements when `True`. Defaults to `False`
    :param int refresh_rate: Maximum number of readings per second. Faster property reads
      will be from the previous reading.

    **Quickstart: Importing and using the BME680**

        Here is an example of using the :class:`BMP680_I2C` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import adafruit_bme680

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()   # uses board.SCL and board.SDA
            bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)

        You need to setup the pressure at sea level

        .. code-block:: python

            bme680.sea_level_pressure = 1013.25

        Now you have access to the :attr:`temperature`, :attr:`gas`, :attr:`relative_humidity`,
        :attr:`pressure` and :attr:`altitude` attributes

        .. code-block:: python

            temperature = bme680.temperature
            gas = bme680.gas
            relative_humidity = bme680.relative_humidity
            pressure = bme680.pressure
            altitude = bme680.altitude

    """

    def __init__(
        self,
        i2c: I2C,
        address: int = 0x77,
        debug: bool = False,
        *,
        refresh_rate: int = 10
    ) -> None:
        """Initialize the I2C device at the 'address' given"""
        from adafruit_bus_device import (  # pylint: disable=import-outside-toplevel
            i2c_device,
        )

        self._i2c = i2c_device.I2CDevice(i2c, address)
        self._debug = debug
        super().__init__(refresh_rate=refresh_rate)

    def _read(self, register: int, length: int) -> bytearray:
        """Returns an array of 'length' bytes from the 'register'"""
        with self._i2c as i2c:
            i2c.write(bytes([register & 0xFF]))
            result = bytearray(length)
            i2c.readinto(result)
            if self._debug:
                print("\t$%02X => %s" % (register, [hex(i) for i in result]))
            return result

    def _write(self, register: int, values: ReadableBuffer) -> None:
        """Writes an array of 'length' bytes to the 'register'"""
        with self._i2c as i2c:
            buffer = bytearray(2 * len(values))
            for i, value in enumerate(values):
                buffer[2 * i] = register + i
                buffer[2 * i + 1] = value
            i2c.write(buffer)
            if self._debug:
                print("\t$%02X <= %s" % (values[0], [hex(i) for i in values[1:]]))


class Adafruit_BME680_SPI(Adafruit_BME680):
    """Driver for SPI connected BME680.

    :param ~busio.SPI spi: SPI device
    :param ~digitalio.DigitalInOut cs: Chip Select
    :param bool debug: Print debug statements when `True`. Defaults to `False`
    :param int baudrate: Clock rate, default is :const:`100000`
    :param int refresh_rate: Maximum number of readings per second. Faster property reads
      will be from the previous reading.


    **Quickstart: Importing and using the BME680**

        Here is an example of using the :class:`BMP680_SPI` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from digitalio import DigitalInOut, Direction
            import adafruit_bme680

        Once this is done you can define your `board.SPI` object and define your sensor object

        .. code-block:: python

            cs = digitalio.DigitalInOut(board.D10)
            spi = board.SPI()
            bme680 = adafruit_bme680.Adafruit_BME680_SPI(spi, cs)

        You need to setup the pressure at sea level

        .. code-block:: python

            bme680.sea_level_pressure = 1013.25

        Now you have access to the :attr:`temperature`, :attr:`gas`, :attr:`relative_humidity`,
        :attr:`pressure` and :attr:`altitude` attributes

        .. code-block:: python

            temperature = bme680.temperature
            gas = bme680.gas
            relative_humidity = bme680.relative_humidity
            pressure = bme680.pressure
            altitude = bme680.altitude

    """

    def __init__(
        self,
        spi: SPI,
        cs: DigitalInOut,
        baudrate: int = 100000,
        debug: bool = False,
        *,
        refresh_rate: int = 10
    ) -> None:
        from adafruit_bus_device import (  # pylint: disable=import-outside-toplevel
            spi_device,
        )

        self._spi = spi_device.SPIDevice(spi, cs, baudrate=baudrate)
        self._debug = debug
        super().__init__(refresh_rate=refresh_rate)

    def _read(self, register: int, length: int) -> bytearray:
        if register != _BME680_REG_STATUS:
            # _BME680_REG_STATUS exists in both SPI memory pages
            # For all other registers, we must set the correct memory page
            self._set_spi_mem_page(register)

        register = (register | 0x80) & 0xFF  # Read single, bit 7 high.
        with self._spi as spi:
            spi.write(bytearray([register]))  # pylint: disable=no-member
            result = bytearray(length)
            spi.readinto(result)  # pylint: disable=no-member
            if self._debug:
                print("\t$%02X => %s" % (register, [hex(i) for i in result]))
            return result

    def _write(self, register: int, values: ReadableBuffer) -> None:
        if register != _BME680_REG_STATUS:
            # _BME680_REG_STATUS exists in both SPI memory pages
            # For all other registers, we must set the correct memory page
            self._set_spi_mem_page(register)
        register &= 0x7F  # Write, bit 7 low.
        with self._spi as spi:
            buffer = bytearray(2 * len(values))
            for i, value in enumerate(values):
                buffer[2 * i] = register + i
                buffer[2 * i + 1] = value & 0xFF
            spi.write(buffer)  # pylint: disable=no-member
            if self._debug:
                print("\t$%02X <= %s" % (values[0], [hex(i) for i in values[1:]]))

    def _set_spi_mem_page(self, register: int) -> None:
        spi_mem_page = 0x00
        if register < 0x80:
            spi_mem_page = 0x10
        self._write(_BME680_REG_STATUS, [spi_mem_page])
