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
 * - Initial version by ProAce based on a fork of Paulvha's sketch
 *********************************************************************
*/

#include "sps30_uart.h"

// Public functions

// Constructor and initializes variables
SPS30_UART::SPS30_UART(void)
{
    _send_buffer_length = 0;                   // Send buffer is still empty
    _receive_buffer_length = 0;                // Recive buffer is still empty
    _SPS30_debug = 0;                          // Debug mode defaults to 0
    _started = false;                          // The sensor has not been started yet
    memset(_reported, 0x1, sizeof(_reported)); // Fill the _reported array with ones
}

// Initialize the communication port, starting of the communication port should happen in main sketch
bool SPS30_UART::begin(Uart *the_uart)
{
    _serial = the_uart;

    return probe();
}

// Enable or disable the printing of sent/response HEX values.
// 0 = no debugging, 1 = sending and receiving data, 2 = 1 + protocol progress
void SPS30_UART::enable_debugging(uint8_t act)
{
    _SPS30_debug = act;
}

// probe probes the SPS30 to see if it is available
bool SPS30_UART::probe()
{
    char buf[32];

    if (get_serial_number(buf, 32) == ERR_OK)
    {
        return true;
    }

    return false;
}

// get_auto_clean_interval reads the interval into a pointer
uint8_t SPS30_UART::get_auto_clean_interval(uint32_t *val)
{
    if (SHDLC_fill_buffer(READ_AUTO_CLEANING) != true)
    {
        return ERR_PARAMETER;
    }

    uint8_t ret = read_from_serial();

    *val = byte_to_U32(&_receive_buffer[SHDLC_DATA_BYTE]); // Parse received data

    return ret;
}

// set_auto_clean_interval sets the interval to a value in seconds
uint8_t SPS30_UART::set_auto_clean_interval(uint32_t val)
{
    // fill buffer to send
    if (SHDLC_fill_buffer(WRITE_AUTO_CLEANING, val) != true)
    {
        return ERR_PARAMETER;
    }

    return read_from_serial();
}

// get_values reads all the sensor values and fills them into a pointer struct
uint8_t SPS30_UART::get_values(struct sps_values *v)
{
    // Has a measurement been started?
    if (_started == false)
    {
        if (start() == false)
        {
            return ERR_CMDSTATE;
        }
    }

    // Fill the sending buffer with the correct command
    if (SHDLC_fill_buffer(READ_MEASURED_VALUE) != true)
    {
        return ERR_PARAMETER;
    }

    // Send the command and read the reply
    uint8_t ret = read_from_serial();

    if (ret != ERR_OK)
    {
        return ret;
    }

    // Check the length of the received message
    if (_receive_buffer[SHDLC_LENGTH_BYTE] != READ_MEASURED_VALUE_LENGTH)
    {
        if (_SPS30_debug)
        {
            printf("%d There aren't enough bytes for all values\n", _receive_buffer[4]);
        }
        return ERR_DATALENGTH;
    }

    memset(v, 0x0, sizeof(struct sps_values)); // Erase the struct

    // Extract the data from the array to the struct
    v->MassPM1 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE]);
    v->MassPM2 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 4]);
    v->MassPM4 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 8]);
    v->MassPM10 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 12]);
    v->NumPM0 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 16]);
    v->NumPM1 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 20]);
    v->NumPM2 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 24]);
    v->NumPM4 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 28]);
    v->NumPM10 = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 32]);
    v->PartSize = byte_to_float(&_receive_buffer[SHDLC_DATA_BYTE + 36]);
}

// Private functions

// get_device_info reads the info to a buffer
uint8_t SPS30_UART::get_device_info(uint8_t type, char *ser, uint8_t len)
{
    if (SHDLC_fill_buffer(type) != true)
    {
        return ERR_PARAMETER;
    }

    uint8_t ret = read_from_serial();

    if (ret != ERR_OK)
    {
        return ret;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        ser[i] = _receive_buffer[SHDLC_DATA_BYTE + i];
        if (ser[i] == NULL) // If the byte is empty the serial number is complete
        {
            break;
        }
    }

    return ERR_OK;
}

// send_commands sends a command to the SPS30
bool SPS30_UART::send_command(uint8_t type)
{
    if ((type == START_FAN_CLEANING) && (_started == false))
    {
        if (_SPS30_debug)
        {
            printf("ERROR : Sensor is not in measurement mode\n");
        }
        return false;
    }

    if (SHDLC_fill_buffer(type) != true)
    {
        return ERR_PARAMETER;
    }

    uint8_t ret = read_from_serial();

    if (ret == ERR_OK)
    {
        switch (type)
        {
        case START_MEASUREMENT:
            _started = true;
            break;

        case STOP_MEASUREMENT:
            _started = false;
            break;

        case RESET:
            _started = false;
            break;
        }
        return true;
    }
    return false;
}

// get_single_value returns a single value read from the sensor
// It acts as a buffer and only if one value is read more than ones it will get new values
float SPS30_UART::get_single_value(uint8_t value)
{
    static struct sps_values v;

    if (value > PartSize) // If the requested value does not exist return -1
    {
        return -1;
    }

    // If the value has already been read
    if (_reported[value])
    {
        // Get new values
        if (get_values(&v) != ERR_OK)
        {
            return -1;
        }
        memset(_reported, 0, sizeof(_reported));
    }

    _reported[value] = 1;

    switch (value)
    {
    case MassPM1:
        return v.MassPM1;
    case MassPM2:
        return v.MassPM2;
    case MassPM4:
        return v.MassPM4;
    case MassPM10:
        return v.MassPM10;
    case NumPM0:
        return v.NumPM0;
    case NumPM1:
        return v.NumPM1;
    case NumPM2:
        return v.NumPM2;
    case NumPM4:
        return v.NumPM4;
    case NumPM10:
        return v.NumPM10;
    case PartSize:
        return v.PartSize;
    }
}

// read_from_serial sends the command in the _send_buffer and reads the response from the SPS30
uint8_t SPS30_UART::read_from_serial()
{
    uint8_t ret;

    _serial->flush(); // Flush anything pending on the set serial

    ret = send_to_serial();

    if (ret != ERR_OK)
    {
        return ret;
    }

    delay(RX_DELAY_MS); // Give the SPS30 some time to respond

    ret = serial_to_buffer();

    if (ret != ERR_OK)
    {
        return ret;
    }

    ret = SHDLC_calc_CRC(_receive_buffer, 1, _receive_buffer_length - 2); // Check the CRC

    if (_receive_buffer[_receive_buffer_length - 1] != ret)
    {
        if (_SPS30_debug)
        {
            printf("CRC error. expected 0x%02X, got 0x%02X\n", _receive_buffer[_receive_buffer_length - 1], ret);
        }
        return ERR_PROTOCOL;
    }

    if (_receive_buffer[SHDLC_STATE_BYTE] != ERR_OK) // Check the state response for errors
    {
        if (_SPS30_debug)
        {
            printf("%x : state error\n", _receive_buffer[SHDLC_STATE_BYTE]);
        }
    }

    return _receive_buffer[SHDLC_STATE_BYTE];
}

// serial_to_buffer reads the serial input into the _receive_buffer and performs byte unstuffing
uint8_t SPS30_UART::serial_to_buffer()
{
    uint32_t start_time = millis();
    bool byte_stuff = false;
    uint8_t i;

    i = 0;

    while (true) // Read the input stream until the last SHDLC_HEADER
    {
        if (millis() - start_time > TIME_OUT) // Prevent deadlock by timing out after a while
        {
            if (_SPS30_debug > 1)
            {
                printf("TimeOut during reading byte %d\n", i);
            }

            return ERR_TIMEOUT;
        }

        if (_serial->available())
        {
            _receive_buffer[i] = _serial->read();

            if (i == 0) // If it is the first byte
            {
                if (_receive_buffer[i] != SHDLC_HEADER) // Check if the received byte is the SHDLC header byte
                {

                    if (_SPS30_debug > 1)
                    {
                        printf("Incorrect Header. Expected 0x7E got 0x02X\n", _receive_buffer[i]);
                    }

                    return ERR_PROTOCOL;
                }
            }
            else // Else parse the bytes
            {
                if (_receive_buffer[i] == SHDLC_STUFFING_BYTE) // If the received byte is a stuffing byte
                {
                    i--;               // Remove the stuffing byte
                    byte_stuff = true; // And the next byte should be unstuffed
                }
                else if (byte_stuff)
                {
                    _receive_buffer[i] = byte_unstuffing(_receive_buffer[i]);
                    byte_stuff = false;
                }
                else if (_receive_buffer[i] == SHDLC_HEADER) // If a trailer byte is received stop the receiving
                {
                    _receive_buffer_length = i;

                    if (_SPS30_debug)
                    {
                        printf("Received: ");
                        for (i = 0; i < _receive_buffer_length + 1; i++)
                        {
                            printf("0x%02X ", _receive_buffer[i]);
                        }
                        printf("length: %d\n\n", _receive_buffer_length);
                    }

                    /* if a board can not handle 115K you get uncontrolled input
                     * that can result in short /wrong messages
                     */
                    if (_receive_buffer_length < 3)
                    {
                        return ERR_PROTOCOL;
                    }

                    return ERR_OK;
                }
            }

            i++;

            if (i > MAXRECVBUFLENGTH)
            {
                if (_SPS30_debug)
                {
                    printf("\nReceive buffer full\n");
                }
                return ERR_PROTOCOL;
            }
        }
    }
}

// send_to_serial sends the _send_buffer over the set serial connection
uint8_t SPS30_UART::send_to_serial()
{
    if (_send_buffer_length == 0)
    {
        return ERR_DATALENGTH;
    }

    if (_SPS30_debug)
    {
        printf("Sending: ");
        for (uint8_t i = 0; i < _send_buffer_length; i++)
        {
            printf(" 0x%02X", _send_buffer[i]);
        }
        printf("\n");
    }

    for (uint8_t i = 0; i < _send_buffer_length; i++)
    {
        _serial->write(_send_buffer[i]); // Send all the bytes over the set serial
    }

    // Indicate that command has been sent by clearing the _send_buffer_length
    _send_buffer_length = 0;

    return ERR_OK;
}

// SHDLC_fill_buffer fills the _send_buffer based on the given command and parameter
bool SPS30_UART::SHDLC_fill_buffer(uint8_t command, uint32_t parameter)
{
    memset(_send_buffer, 0x0, sizeof(_send_buffer));
    _send_buffer_length = 0;

    int i = 0;
    uint8_t tmp;

    _send_buffer[i++] = SHDLC_HEADER; // Start message with a header
    _send_buffer[i++] = 0x0;          // Add the address of the SPS30 which is zero
    _send_buffer[i++] = command;      // Add the command

    switch (command)
    {
    case START_MEASUREMENT:
        _send_buffer[i++] = 2; // Add the data length
        _send_buffer[i++] = 0x1;
        _send_buffer[i++] = 0x3;
        break;

    case STOP_MEASUREMENT:
    case READ_MEASURED_VALUE:
    case START_FAN_CLEANING:
    case RESET:
        _send_buffer[i++] = 0; // Add the data length
        break;

    case READ_DEVICE_PRODUCT_NAME:
    case READ_DEVICE_ARTICLE_CODE:
    case READ_DEVICE_SERIAL_NUMBER:
        _send_buffer[2] = READ_DEVICE_INFO;
        _send_buffer[i++] = 1;              // Add the data length
        _send_buffer[i++] = command & 0x0F; // On these reads the command is used as data bytes
        break;

    case READ_AUTO_CLEANING:
        _send_buffer[2] = AUTO_CLEANING_INTERVAL;
        _send_buffer[i++] = 1; // Add the data length
        _send_buffer[i++] = 0; // Add a subcommand, this value must be set to 0x00
        break;

    case WRITE_AUTO_CLEANING:
        _send_buffer[2] = AUTO_CLEANING_INTERVAL;
        _send_buffer[i++] = 5; // Add the data length
        _send_buffer[i++] = 0; // Add a subcommand, this value must be set to 0x00

        tmp = parameter >> 24 & 0xFF; // Change order depending on the endianess
        i = byte_stuffing(tmp, i);
        tmp = parameter >> 16 & 0xFF;
        i = byte_stuffing(tmp, i);
        tmp = parameter >> 8 & 0xFF;
        i = byte_stuffing(tmp, i);
        tmp = parameter & 0xFF;
        i = byte_stuffing(tmp, i);
        break;

    default:
        return false;
    }

    // Add the CRC and check it for byte stuffing
    tmp = SHDLC_calc_CRC(_send_buffer, 1, i);
    i = byte_stuffing(tmp, i);

    _send_buffer[i] = SHDLC_HEADER; // Add a trailer byte
    _send_buffer_length = ++i;      // Store the length off the _send_buffer

    return true;
}

// SHDLC_calc_CRC calculates the CRC, the CRC is calculated by taking the inverse of the LSB of the sum of all the bytes between the headers
uint8_t SPS30_UART::SHDLC_calc_CRC(uint8_t *buf, uint8_t first, uint8_t last)
{
    uint32_t ret = 0;

    for (uint8_t i = first; i <= last; i++)
    {
        ret += buf[i];
    }

    return ~(ret & 0xff);
}

// byte_stuffing checks if a byte needs stuffing and stuffs it
int SPS30_UART::byte_stuffing(uint8_t b, int offset)
{
    uint8_t x = 0;

    switch (b)
    {
    case 0x11:
        x = 0x31;
        break;

    case 0x13:
        x = 0x33;
        break;

    case 0x7d:
        x = 0x5d;
        break;

    case 0x7e:
        x = 0x5e;
        break;
    }

    if (x == 0)
        _send_buffer[offset++] = b;
    else
    {
        _send_buffer[offset++] = 0x7D;
        _send_buffer[offset++] = x;
    }

    return offset;
}

// byte_unstuffing unstuffs stuffed bytes
uint8_t SPS30_UART::byte_unstuffing(uint8_t b)
{
    switch (b)
    {
    case 0x31:
        return (0x11);
    case 0x33:
        return (0x13);
    case 0x5d:
        return (0x7d);
    case 0x5e:
        return (0x7e);

    default:
        if (_SPS30_debug > 1)
        {
            printf("Incorrect byte Unstuffing. Got: 0x%02X\n", b);
        }
        return NULL;
    }
}

// byte_to_float translates a byte array to a float
float SPS30_UART::byte_to_float(uint8_t *array)
{
    uint32_t value;
    float float_value;

    for (byte i = 0; i < 4; i++)
    {
        value += (uint32_t)array[3-i] << (i * 8);
    }

    memcpy(&float_value, &value, sizeof(float_value));

    return float_value;
}

// byte_to_U32 translates a byte array to an uint32_t
uint32_t SPS30_UART::byte_to_U32(uint8_t *array)
{
    uint32_t value;

    for (byte i = 0; i < 4; i++)
    {
        value += (uint32_t)array[3-i] << (i * 8);
    }

    return value;
}