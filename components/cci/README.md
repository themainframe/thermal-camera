# Why is there a custom I2C driver?

You're thinking "_but the ESP32 IDF has support for I2C! Is Damo crazy? Why would he write his own?_"

And you'd be right. The ESP32 IDF does indeed have I2C support, but at time of writing, it was [rather broken](https://github.com/espressif/esp-idf/issues/1503) 😟 I ran into lots of issues that caused the `thermal-camera` firmware to become quite unstable whenever I2C was involved.

This implementation is basic and really aimed at satisfying the requirements of the CCI (Camera Control Interface) not a general-purpose I2C library.
