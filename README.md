# Sensirion SPS30

This driver enables you to communicate with the SPS30 sensor over any UART or TwoWire(I2C) serial port. The UART part has been tested, the TwoWire is still untested however.

## The basics

The library automatically switches between UART and TwoWire mode based on the  port passed to the begin function.

```cpp
    bool begin(Stream *the_uart = &Serial1); // If user doesn't specify Serial1 will be used
    bool begin(TwoWire *the_wire);
```

The begin function returns true if the probe() function succeeded. If it succeeds a SPS30 sensor is available.

All the other functions are explained via the given examples