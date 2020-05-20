/**
 * SPS30 - UART Library Header file
 *
 * Copyright (c) January 2019, Paul van Haastrecht
 *
 * All rights reserved.
 *
 * Development environment specifics:
 * Arduino IDE 1.9
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **********************************************************************
 * Version 1.0   / March 2020
 * - Initial version by ProAce based on a fork of Paulvha's build
 *********************************************************************
*/

#ifndef SPS30_H
#define SPS30_H

#include "Arduino.h" // Needed for Stream
#include <Wire.h>

#define MAX_RECEIVE_BUFFER_LENGTH 80 // ~Max response length with byte stuffing
#define MAX_DATA_LENGTH 40           // Max data length = 40

#define I2C_CRC_POLYNOMIAL 0x31
#define I2C_CRC_INITIALIZATION 0xFF

#define I2C_LENGTH 32

#if defined BUFFER_LENGTH // Arduino  & ESP8266 & Softwire
#undef I2C_LENGTH
#define I2C_LENGTH BUFFER_LENGTH
#endif

#if defined I2C_BUFFER_LENGTH // ESP32
#undef I2C_LENGTH
#define I2C_LENGTH I2C_BUFFER_LENGTH
#endif

#if defined ARDUINO_ARCH_SAMD || defined ARDUINO_ARCH_SAM21D // Depending on definition in wire.h (RingBufferN<256> rxBuffer;)
#undef I2C_LENGTH
#define I2C_LENGTH 256
#endif

// Struct containing sensor values
typedef struct Measurements
{
    float MassPM1;  // Mass Concentration PM1.0 [μg/m3]
    float MassPM2;  // Mass Concentration PM2.5 [μg/m3]
    float MassPM4;  // Mass Concentration PM4.0 [μg/m3]
    float MassPM10; // Mass Concentration PM10 [μg/m3]
    float NumPM0;   // Number Concentration PM0.5 [#/cm3]
    float NumPM1;   // Number Concentration PM1.0 [#/cm3]
    float NumPM2;   // Number Concentration PM2.5 [#/cm3]
    float NumPM4;   // Number Concentration PM4.0 [#/cm3]
    float NumPM10;  // Number Concentration PM4.0 [#/cm3]
    float PartSize; // Typical Particle Size [μm]
};

// The message struct contains all the relevant fields for I2C and SHDLC messages to the SPS30
typedef struct Message
{
    uint16_t address;
    uint16_t command;
    uint8_t state;
    uint8_t length;
    uint8_t read_length;
    uint8_t data[MAX_DATA_LENGTH];
};

// The version struct contains all the version information.
typedef struct Version
{
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t hardware;
    uint8_t SHDLC_major;
    uint8_t SHDLC_minor;
};

// Enum for retrieval of single values
enum values
{
    MassPM1 = 1,
    MassPM2,
    MassPM4,
    MassPM10,
    NumPM0,
    NumPM1,
    NumPM2,
    NumPM4,
    NumPM10,
    PartSize
};

enum status
{
    SPEED,
    LASER,
    FAN
};

enum commands
{
    START_MEASUREMENT,
    STOP_MEASUREMENT,
    READ_DATA_READY,
    READ_MEASURED_VALUE,
    SLEEP,
    WAKE_UP,
    START_FAN_CLEANING,
    RESET,
    READ_DEVICE_PRODUCT_TYPE,
    READ_DEVICE_SERIAL_NUMBER,
    READ_VERSION,
    READ_STATUS_REGISTER,
    AUTO_CLEANING_INTERVAL,
    READ_AUTO_CLEANING,
    WRITE_AUTO_CLEANING
};

enum SHDLC_commands
{
    SHDLC_START_MEASUREMENT = 0x00,
    SHDLC_STOP_MEASUREMENT = 0x01,
    SHDLC_READ_MEASURED_VALUE = 0x03,
    SHDLC_SLEEP = 0x10,   // Software version 2.0
    SHDLC_WAKE_UP = 0x11, // Software version 2.0
    SHDLC_START_FAN_CLEANING = 0x56,
    SHDLC_RESET = 0xD3,

    SHDLC_READ_DEVICE_INFO = 0xD0,          // Generic device request
    SHDLC_READ_DEVICE_PRODUCT_TYPE = 0x00,  // Request product type
    SHDLC_READ_DEVICE_SERIAL_NUMBER = 0x03, // Request serial number

    SHDLC_READ_VERSION = 0xD1,
    SHDCL_READ_STATUS_REGISTER = 0xD2, // Software version 2.2

    SHDLC_AUTO_CLEANING_INTERVAL = 0x80, // Generic autoclean request

    SHDLC_HEADER = 0x7E,        // Header & trailer byte
    SHDLC_STUFFING_BYTE = 0x7D, // Stuffing byte
    SHDLC_ADDRESS_BYTE = 0x01,
    SHDLC_COMMAND_BYTE = 0x02,
    SHDLC_STATE_BYTE = 0x03,                // Byte storing the state message
    SHDLC_LENGTH_BYTE = 0x04,               // Byte storing the message length
    SHDLC_DATA_BYTE = 0x05,                 // First byte containing data
    SHDLC_READ_MEASURED_VALUE_LENGTH = 0x28 // A read measurement message is 40 bytes long
};

enum I2C_commands
{
    I2C_START_MEASUREMENT = 0x0010,
    I2C_STOP_MEASUREMENT = 0x0104,
    I2C_READ_DATA_READY = 0x0202,
    I2C_READ_MEASURED_VALUE = 0x0300,
    I2C_SLEEP = 0x1001,   // Software version 2.0
    I2C_WAKE_UP = 0x1103, // Software version 2.0
    I2C_START_FAN_CLEANING = 0x5607,
    I2C_READ_WRITE_AUTO_CLEANING = 0x8004,
    I2C_READ_PRODUCT_TYPE = 0xD002,
    I2C_READ_SERIAL_NUMBER = 0xD033,
    I2C_READ_VERSION = 0xD100,
    I2C_READ_DEVICE_STATUS_REGISTER = 0xD206,  // Software version 2.2
    I2C_CLEAR_DEVICE_STATUS_REGISTER = 0xD210, // Software version 2.0
    I2C_RESET = 0xD304
};

#define TIME_OUT 200   // Timeout to prevent deadlock read
#define RX_DELAY_MS 20 // Wait between write and read

class SPS30
{
public:
    SPS30(void);

    boolean begin(Stream *the_uart = &Serial1); // If user doesn't specify Serial1 will be used
    boolean begin(TwoWire *the_wire);

    void enable_debugging(Stream *debug = &Serial);
    void disable_debugging();

    boolean probe();
    boolean reset();
    boolean start();
    boolean stop();
    boolean clean();
    boolean sleep();
    boolean wake_up();

    uint32_t get_auto_clean_interval();
    boolean set_auto_clean_interval(uint32_t val);

    boolean get_serial_number(char *ser, uint8_t len) { return get_device_info(SHDLC_READ_DEVICE_SERIAL_NUMBER, ser, len); }
    boolean get_product_type(char *ser, uint8_t len) { return get_device_info(SHDLC_READ_DEVICE_PRODUCT_TYPE, ser, len); }

    boolean read_version(Version *response);

    boolean read_speed_status(boolean *error, boolean clear = false) { return get_device_status(SPEED, error, clear); }
    boolean read_fan_status(boolean *error, boolean clear = false) { return get_device_status(FAN, error, clear); }
    boolean read_laser_status(boolean *error, boolean clear = false) { return get_device_status(LASER, error, clear); }

    boolean get_values(Measurements *v);

    float get_mass_PM1() { return (get_single_value(MassPM1)); }
    float get_mass_PM2() { return (get_single_value(MassPM2)); }
    float get_mass_PM4() { return (get_single_value(MassPM4)); }
    float get_mass_PM10() { return (get_single_value(MassPM10)); }
    float get_num_PM0() { return (get_single_value(NumPM0)); }
    float get_num_PM1() { return (get_single_value(NumPM1)); }
    float get_num_PM2() { return (get_single_value(NumPM2)); }
    float get_num_PM4() { return (get_single_value(NumPM4)); }
    float get_num_PM10() { return (get_single_value(NumPM10)); }
    float get_part_size() { return (get_single_value(PartSize)); }

private:
    boolean _i2c_mode = false;       // If it is in I2C mode, it isn't in UART mode and vice versa
    boolean _i2c_max_length = false; // Do we need to account for a small I2C buffer?

    boolean _SPS30_debug = false; // Program debug level
    boolean _started = false;     // Indicate the measurement has started
    uint8_t _reported[11];        // Use as cache indicator single value

    boolean send_command(Message *response, uint8_t command, uint32_t parameter = 0);
    float get_single_value(uint8_t value);
    boolean get_device_info(uint8_t command, char *ser, uint8_t len);
    boolean get_device_status(uint8_t command, boolean *error, boolean clear);

    //I2C functions
    boolean I2C_send_command(Message *response, uint8_t command, uint32_t parameter = 0);
    boolean I2C_read(Message *message);
    boolean I2C_send(Message *message);

    boolean I2C_create_command(Message *message, uint8_t command, uint32_t parameter = 0);
    uint8_t I2C_calculate_CRC(uint8_t *data);

    // SHDLC functions
    boolean SHDLC_send_command(Message *response, uint8_t command, uint32_t parameter = 0);
    boolean SHDLC_read(Message *message);
    boolean SHDLC_send(Message *message);

    boolean SHDLC_create_command(Message *message, uint8_t command, uint32_t parameter = 0);
    uint8_t SHDLC_calculate_CRC(Message *message, boolean received);

    uint8_t byte_stuffing(uint8_t *buffer, uint8_t value, uint8_t offset);
    uint8_t byte_unstuffing(uint8_t value);

    float byte_to_float(uint8_t *buffer);
    uint32_t byte_to_U32(uint8_t *buffer);

    Stream *_serial;
    Stream *_debug;
    TwoWire *_i2c;
};
#endif
