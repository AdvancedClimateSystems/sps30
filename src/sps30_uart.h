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
#include "printf.h"

// Auto detect that some boards have low memory. (like Uno)

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega16U4__) || (__AVR_ATmega32U4__)
#error "The UART variant of this library won't work on this device due to low memory"
#endif // AVR definition check

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

enum error_codes
{
    ERR_OK = 0x00,
    ERR_DATALENGTH = 0X01,
    ERR_UNKNOWNCMD = 0x02,
    ERR_ACCESSRIGHT = 0x03,
    ERR_PARAMETER = 0x04,
    ERR_OUTOFRANGE = 0x28,
    ERR_CMDSTATE = 0x43,
    ERR_TIMEOUT = 0x50,
    ERR_PROTOCOL = 0x51
};

#define MAXRECVBUFLENGTH 80 // Max response length = 39 bytes without byte stuffing

enum commands
{
    START_MEASUREMENT = 0x00,
    STOP_MEASUREMENT = 0x01,
    READ_MEASURED_VALUE = 0x03,
    READ_MEASURED_VALUE_LENGTH = 0x28, // A read measurement message is 40 bytes long
    START_FAN_CLEANING = 0x56,
    RESET = 0xD3,

    READ_DEVICE_INFO = 0xD0, // Generic device request
    READ_DEVICE_PRODUCT_NAME = 0xF1,
    READ_DEVICE_ARTICLE_CODE = 0xF2,
    READ_DEVICE_SERIAL_NUMBER = 0xF3,

    AUTO_CLEANING_INTERVAL = 0x80, // Generic autoclean request
    READ_AUTO_CLEANING = 0x81,     // Read autoclean
    WRITE_AUTO_CLEANING = 0x82,    // Write autoclean

    SHDLC_HEADER = 0x7E,        // Header & trailer byte
    SHDLC_STUFFING_BYTE = 0x7D, // Stuffing byte
    SHDLC_STATE_BYTE = 0x03,    // Byte storing the state message
    SHDLC_LENGTH_BYTE = 0x04,   // Byte storing the message length
    SHDLC_DATA_BYTE = 0x05      // First byte containing data
};

#define TIME_OUT 200   // Timeout to prevent deadlock read
#define RX_DELAY_MS 20 // Wait between write and read

class SPS30_UART
{
public:
    SPS30_UART(void);

    bool begin(Uart *the_uart = &Serial1); // If user doesn't specify Serial1 will be used

    void enable_debugging(uint8_t act);

    bool probe();
    bool reset() { return (send_command(RESET)); }
    bool start() { return (send_command(START_MEASUREMENT)); }
    bool stop() { return (send_command(STOP_MEASUREMENT)); }
    bool clean() { return (send_command(START_FAN_CLEANING)); }

    uint8_t get_auto_clean_interval(uint32_t *val);
    uint8_t set_auto_clean_interval(uint32_t val);

    uint8_t get_serial_number(char *ser, uint8_t len) { return (get_device_info(READ_DEVICE_SERIAL_NUMBER, ser, len)); }
    uint8_t get_article_code(char *ser, uint8_t len) { return (get_device_info(READ_DEVICE_ARTICLE_CODE, ser, len)); }
    uint8_t get_product_name(char *ser, uint8_t len) { return (get_device_info(READ_DEVICE_PRODUCT_NAME, ser, len)); }

    uint8_t get_values(struct sps_values *v);

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
    uint8_t _receive_buffer[MAXRECVBUFLENGTH]; // Buffers
    uint8_t _send_buffer[10];
    uint8_t _receive_buffer_length;
    uint8_t _send_buffer_length;

    int _SPS30_debug;      // Program debug level
    bool _started;         // Indicate the measurement has started
    uint8_t _reported[11]; // Use as cache indicator single value

    uint8_t get_device_info(uint8_t type, char *ser, uint8_t len);
    bool send_command(uint8_t type);
    float get_single_value(uint8_t value);

    uint8_t read_from_serial();
    uint8_t serial_to_buffer();
    uint8_t send_to_serial();

    bool SHDLC_fill_buffer(uint8_t command, uint32_t parameter = 0);
    uint8_t SHDLC_calc_CRC(uint8_t *buf, uint8_t first, uint8_t last);

    int byte_stuffing(uint8_t b, int off);
    uint8_t byte_unstuffing(uint8_t b);

    float byte_to_float(uint8_t *array);
    uint32_t byte_to_U32(uint8_t *array);

    Uart *_serial;
};
#endif
