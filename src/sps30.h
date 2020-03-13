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

// Struct containing sensor values
typedef struct sps_values
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

typedef struct Message
{
    uint16_t address;
    uint16_t command;
    uint8_t state;
    uint8_t length;
    uint8_t data[MAX_DATA_LENGTH];
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

enum commands
{
    START_MEASUREMENT,
    STOP_MEASUREMENT,
    READ_MEASURED_VALUE,
    START_FAN_CLEANING,
    RESET,
    READ_DEVICE_INFO,
    READ_DEVICE_PRODUCT_NAME,
    READ_DEVICE_ARTICLE_CODE,
    READ_DEVICE_SERIAL_NUMBER,
    AUTO_CLEANING_INTERVAL,
    READ_AUTO_CLEANING,
    WRITE_AUTO_CLEANING
};

enum SHDLC_commands
{
    SHDLC_START_MEASUREMENT = 0x00,
    SHDLC_STOP_MEASUREMENT = 0x01,
    SHDLC_READ_MEASURED_VALUE = 0x03,
    SHDLC_READ_MEASURED_VALUE_LENGTH = 0x28, // A read measurement message is 40 bytes long
    SHDLC_START_FAN_CLEANING = 0x56,
    SHDLC_RESET = 0xD3,

    SHDLC_READ_DEVICE_INFO = 0xD0, // Generic device request
    SHDLC_READ_DEVICE_PRODUCT_NAME = 0xF1,
    SHDLC_READ_DEVICE_ARTICLE_CODE = 0xF2,
    SHDLC_READ_DEVICE_SERIAL_NUMBER = 0xF3,

    SHDLC_AUTO_CLEANING_INTERVAL = 0x80, // Generic autoclean request
    SHDLC_READ_AUTO_CLEANING = 0x81,     // Read autoclean
    SHDLC_WRITE_AUTO_CLEANING = 0x82,    // Write autoclean

    SHDLC_HEADER = 0x7E,        // Header & trailer byte
    SHDLC_STUFFING_BYTE = 0x7D, // Stuffing byte
    SHDLC_ADDRESS_BYTE = 0x01,
    SHDLC_COMMAND_BYTE = 0x02,
    SHDLC_STATE_BYTE = 0x03,  // Byte storing the state message
    SHDLC_LENGTH_BYTE = 0x04, // Byte storing the message length
    SHDLC_DATA_BYTE = 0x05    // First byte containing data
};

#define TIME_OUT 200   // Timeout to prevent deadlock read
#define RX_DELAY_MS 20 // Wait between write and read

class SPS30
{
public:
    SPS30(void);

    bool begin(Stream *the_uart = &Serial1); // If user doesn't specify Serial1 will be used
    bool begin(TwoWire *the_wire = &Wire);

    void enable_debugging(Stream *debug = &Serial);
    void disable_debugging();

    bool probe();
    bool reset();
    bool start();
    bool stop();
    bool clean();

    uint32_t get_auto_clean_interval();
    bool set_auto_clean_interval(uint32_t val);

    bool get_serial_number(char *ser, uint8_t len) { return (get_device_info(READ_DEVICE_SERIAL_NUMBER, ser, len)); }
    bool get_article_code(char *ser, uint8_t len) { return (get_device_info(READ_DEVICE_ARTICLE_CODE, ser, len)); }
    bool get_product_name(char *ser, uint8_t len) { return (get_device_info(READ_DEVICE_PRODUCT_NAME, ser, len)); }

    bool get_values(struct sps_values *v);

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
    bool _i2c_mode;

    bool _SPS30_debug = false; // Program debug level
    bool _started = false;     // Indicate the measurement has started
    uint8_t _reported[11];     // Use as cache indicator single value

    bool send_command(Message *response, uint8_t command, uint32_t parameter = 0);
    bool get_device_info(uint8_t type, char *ser, uint8_t len);
    float get_single_value(uint8_t value);

    //I2C functions

    // SHDLC functions
    bool SHDLC_send_command(Message *response, uint8_t command, uint32_t parameter = 0);
    bool SHDLC_read(Message *message);
    bool SHDLC_send(Message *message);

    bool SHDLC_create_command(Message *message, uint8_t command, uint32_t parameter = 0);
    uint8_t SHDLC_calculate_CRC(Message *message, bool received);

    uint8_t byte_stuffing(uint8_t *buffer, uint8_t value, uint8_t offset);
    uint8_t byte_unstuffing(uint8_t value);

    float byte_to_float(uint8_t *buffer);
    uint32_t byte_to_U32(uint8_t *buffer);

    Stream *_serial;
    Stream *_debug;
    TwoWire *_i2c;
};
#endif
