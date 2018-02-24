# `thermal-camera`

Firmware for my hand-rolled LWIR camera, based on ESP32 and FLIR Lepton 3.

The thermal camera driver is mostly ported from my previous [leptonic](https://github.com/themainframe/leptonic) code.

For (much) more information, see my blog post on [damow.net](https://damow.net/building-a-thermal-camera).

I compiled my firmware against the [ESP32 IoT Development Framework](https://github.com/espressif/esp-idf), release `v3.0-rc1` with toolchain version 1.22.0-75-gbaf03c2. The compilation should work fine on any platform, but for reference I had success on Mac OS X and Linux.

## Building

To build, check out this repository first.

Install the appropriate ESP32 Extensa toolchain for your system - see [https://esp-idf.readthedocs.io/en/v3.0-rc1/get-started/index.html](https://esp-idf.readthedocs.io/en/v3.0-rc1/get-started/index.html). Make sure you're looking at the v3.0-rc1 version documentation, otherwise the wrong toolchain links will be provided. It's critical that the compilation is performed with the correct toolchain.

Also check out the ESP32 IoT Development Framework (IDF):

    git clone https://github.com/espressif/esp-idf.git esp-idf
    cd esp-idf
    git checkout v3.0-rc1
    git submodule update --init --recursive

Set the `IDF_PATH` environment variable to point to the path where you checked out the IDF above.

In the `thermal-camera` directory, set up your serial port settings with `make menuconfig`.

Build with `make`.

Flash with `make flash`.

