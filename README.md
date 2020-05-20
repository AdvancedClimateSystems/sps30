# Sensirion SPS30

This library enabled the use of the SPS30 sensor over a UART or I2C interface. It offers easier initiation, easier to read data structures, and it works with less delays making it faster in use.

## The basics

The library can be used in the same way for both interface options. The only difference will be how you start the sensor.

Starting the sensor in UART mode

```cpp
SPS30 sps30;

void setup()
{
    Serial.begin(115200); // Start the Serial port you want to use at 115200 bps
    sps30.begin(&Serial); // Begin the sensor by passing along the address of the Serial port you're using.
}
```

Starting the sensor in I2C mode

```cpp
SPS30 sps30;

void setup()
{
    Wire.begin(); // Start the I2C port you want to use
    sps30.begin(&Wire); // Begin the sensor by passing along the address of the I2C port you're using.
}
```

A function will return true or false based on the succes of the read/write operations. Data will be passed back via the pointer values.

## Changelog

### 1.0 Port from Paulvha

This is the initial edit from Paulvha's library. The following things were changed:

- Change the way the different interfaces are handled
- Change the way data is handled
- Remove unnecesary delays
- Add a way to easily redefine the Debug serial interface

### 1.1 Update to the new SPS30 software version

The new versions should only be used when you're certain that your SPS30 has the newer firmware version (2.0 / 2.2).

- Add sleep/wake_up function
- Add status functions
- Remove get_article_code()
- Rename get_product_name() to get_product_type()
- Add get_product_type() for the I2C mode
- Updated examples
- Renamed sps_values struct to Measurements
