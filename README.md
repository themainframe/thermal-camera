# `thermal-camera`

Firmware for my hand-rolled LWIR camera, based on ESP32 and FLIR Lepton 3.

The thermal camera driver is mostly ported from my previous [leptonic](https://github.com/themainframe/leptonic) code.

For (much) more information, see my blog post on [damow.net](https://damow.net/building-a-thermal-camera).

I compiled my firmware against the [ESP32 IoT Development Framework](https://github.com/espressif/esp-idf), commit `6cc8099` with toolchain version 1.22.0-73-ge28a011.

## Building

To build, check out this repository first.

Install the appropriate ESP32 Extensa toolchain for your system - see [https://esp-idf.readthedocs.io/en/latest/get-started/index.html](https://esp-idf.readthedocs.io/en/latest/get-started/index.html).

Also check out the ESP32 IoT Development Framework (IDF) (see above).

Set the `IDF_PATH` environment variable to point to the path where you checked out the IDF.

Set up your serial port settings with `make menuconfig`.

Build with `make`.

Flash with `make flash`.

