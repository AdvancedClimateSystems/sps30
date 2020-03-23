/**
 * SPS30 - UART Library Header file
 *
 * Copyright (c) January 2019, Paul van Haastrecht
 *
 * All rights reserved->
 *
 * Development environment specifics:
 * Arduino IDE 1->9
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE-> See the
 * GNU General Public License for more details->
 *
 * You should have received a copy of the GNU General Public License
 * along with this program->  If not, see <http://www->gnu->org/licenses/>->
 *
 **********************************************************************
 * Version 1->0   / March 2020
 * - Initial version by ProAce based on a fork of Paulvha's sketch
 *********************************************************************
*/

#include "sps30.h"

// Public functions

// Constructor and initializes variables
SPS30::SPS30(void)
{
    memset(_reported, 0x1, sizeof(_reported)); // Fill the _reported array with ones

    if (I2C_LENGTH >= 64)
    {
        _i2c_max_length = true;
    }
}

// Initialize the communication port, starting of the communication port should happen in main sketch
bool SPS30::begin(Stream *the_uart)
{
    _serial = the_uart;
    _i2c_mode = false;

    return probe();
}

// Initialize the communication port, starting of the communication port should happen in main sketch
bool SPS30::begin(TwoWire *the_wire)
{
    _i2c = the_wire;
    _i2c_mode = true;

    return probe();
}

// Enable debugging on a Stream object
void SPS30::enable_debugging(Stream *debug)
{
    _SPS30_debug = true;
    _debug = debug;
}

void SPS30::disable_debugging()
{
    _SPS30_debug = false;
}

// probe probes the SPS30 to see if it is available
bool SPS30::probe()
{
    char buf[32];
    return get_serial_number(buf, 32);
}

bool SPS30::reset()
{
    Message response;
    if (!send_command(&response, RESET))
    {
        return false;
    }

    _started = false;
    return true;
}

bool SPS30::start()
{
    Message response;
    if (!send_command(&response, START_MEASUREMENT))
    {
        return false;
    }

    _started = true;
    return true;
}

bool SPS30::stop()
{
    Message response;
    if (!send_command(&response, STOP_MEASUREMENT))
    {
        return false;
    }

    _started = false;
    return true;
}

bool SPS30::clean()
{
    Message response;
    if (!send_command(&response, SHDLC_START_FAN_CLEANING))
    {
        return false;
    }

    return true;
}

// get_auto_clean_interval reads the interval into a pointer
uint32_t SPS30::get_auto_clean_interval()
{
    Message response;

    if (!send_command(&response, READ_AUTO_CLEANING))
    {
        return 0;
    }

    return byte_to_U32(response.data);
}

// set_auto_clean_interval sets the interval to a value in seconds
bool SPS30::set_auto_clean_interval(uint32_t val)
{
    Message response;

    return send_command(&response, WRITE_AUTO_CLEANING, val);
}

// get_values reads all the sensor values and fills them into a pointer struct
bool SPS30::get_values(struct sps_values *v)
{
    // Has a measurement been started?
    if (_started == false)
    {
        if (start() == false)
        {
            return false;
        }
    }

    Message response;

    if (!send_command(&response, READ_MEASURED_VALUE))
    {
        return false;
    }

    // Check the length of the received message
    if (response.length != SHDLC_READ_MEASURED_VALUE_LENGTH)
    {
        if (_SPS30_debug)
        {
            _debug->print(response.length);
            _debug->println(" There aren't enough bytes for all values");
        }
        return false;
    }

    memset(v, 0x0, sizeof(struct sps_values)); // Erase the struct

    // Extract the data from the array to the struct
    v->MassPM1 = byte_to_float(&response.data[0]);
    v->MassPM2 = byte_to_float(&response.data[4]);
    v->MassPM4 = byte_to_float(&response.data[8]);
    v->MassPM10 = byte_to_float(&response.data[12]);
    if (!_i2c_mode || _i2c_max_length > 20)
    {
        v->NumPM0 = byte_to_float(&response.data[16]);
        v->NumPM1 = byte_to_float(&response.data[20]);
        v->NumPM2 = byte_to_float(&response.data[24]);
        v->NumPM4 = byte_to_float(&response.data[28]);
        v->NumPM10 = byte_to_float(&response.data[32]);
        v->PartSize = byte_to_float(&response.data[36]);
    }
}

// Private functions

// get_device_info reads the info to a buffer
bool SPS30::get_device_info(uint8_t command, char *ser, uint8_t len)
{
    Message response;

    if (!send_command(&response, command))
    {
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        ser[i] = response.data[i];
        if (ser[i] == NULL) // If the byte is empty the serial number is complete
        {
            break;
        }
    }

    return true;
}

// send_commands sends a command to the SPS30
bool SPS30::send_command(Message *response, uint8_t command, uint32_t parameter)
{
    if ((command == START_FAN_CLEANING) && (_started == false))
    {
        if (_SPS30_debug)
        {
            _debug->println("ERROR : Sensor is not in measurement mode");
        }
        return false;
    }

    if (_i2c_mode)
    {
        return I2C_send_command(response, command, parameter);
    }
    else
    {
        return SHDLC_send_command(response, command, parameter);
    }
}

// get_single_value returns a single value read from the sensor
// It acts as a buffer and only if one value is read more than ones it will get new values
float SPS30::get_single_value(uint8_t value)
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
        if (get_values(&v) != 0)
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

bool SPS30::I2C_send_command(Message *response, uint8_t command, uint32_t parameter)
{
    if (command == READ_DEVICE_PRODUCT_NAME) // The I2C doesn't have a read device product name
    {
        return true;
    }

    I2C_create_command(response, command, parameter);

    if (!I2C_send(response))
    {
        return false;
    }

    delay(RX_DELAY_MS); // Give the SPS30 some time to respond

    if (response->read_length != 0)
    {
        if (!I2C_read(response))
        {
            return false;
        }
    }

    return true;
}

bool SPS30::I2C_read(Message *message)
{
    // Request amount data from the sensor with CRC's
    _i2c->requestFrom(message->address, (uint8_t)(message->read_length / 2 * 3));

    int i = 0;
    uint8_t data[3];

    message->length = 0;

    while (_i2c->available())
    {
        _debug->print("HI");
        data[i++] = _i2c->read();

        if (i = 3)
        {
            uint8_t crc = I2C_calculate_CRC(data);
            if (data[2] != crc)
            {
                if (_SPS30_debug)
                {
                    _debug->print("I2C CRC error: Expected ");
                    _debug->print(data[2]);
                    _debug->print(" calculated ");
                    _debug->println(crc);
                }
                return false;
            }

            message->data[message->length++] = data[0];
            message->data[message->length++] = data[1];

            i = 0;

            if (message->length >= message->read_length)
            {
                break;
            }
        }
    }

    if (message->length == 0)
    {
        if (_SPS30_debug)
        {
            _debug->println("Error: Received NO bytes");
        }
        return false;
    }

    if (message->length == message->read_length)
    {
        return true;
    }

    if (_SPS30_debug)
    {
        _debug->print("Error: Expected bytes : ");
        _debug->print(message->read_length);
        _debug->print(", Received bytes ");
        _debug->println(message->length);
    }

    return false;
}

bool SPS30::I2C_send(Message *message)
{
    if (_SPS30_debug)
    {
        _debug->print("I2C Sending: ");
        _debug->print(message->address, HEX);
        _debug->print(" ");
        _debug->print(message->command, HEX);

        for (uint8_t i = 0; i < message->length; i++)
        {
            _debug->print(" ");
            _debug->print(message->data[i], HEX);
        }

        _debug->println("");
    }

    _i2c->beginTransmission(message->address);
    _i2c->write((message->command >> 8) & 0x00FF);
    _i2c->write((message->command) & 0xFF);
    _i2c->write(message->data, message->length);

    if (!_i2c->endTransmission())
    {
        return false;
    }

    return true;
}

bool SPS30::I2C_create_command(Message *message, uint8_t command, uint32_t parameter)
{
    int i = 0;
    message->address = 0x69;
    message->length = 0;
    message->read_length = parameter;

    switch (command)
    {
    case START_MEASUREMENT:
        message->command = I2C_START_MEASUREMENT;
        message->length = 3;       // Add the data length
        message->data[i++] = 0x03; // Measurement mode
        message->data[i++] = 0x00; // Dummy byte
        message->data[i++] = I2C_calculate_CRC(message->data);
        break;

    case STOP_MEASUREMENT:
        message->command = I2C_STOP_MEASUREMENT;
        break;

    case READ_DATA_READY:
        message->command = I2C_READ_DATA_READY;
        message->read_length = 2;
        break;

    case READ_MEASURED_VALUE:
        message->command = I2C_READ_MEASURED_VALUE;
        message->read_length = 40;
        break;

    case START_FAN_CLEANING:
        message->command = I2C_START_FAN_CLEANING;
        break;

    case RESET:
        message->command = I2C_RESET;
        break;

    case READ_DEVICE_ARTICLE_CODE:
        message->command = I2C_READ_DEVICE_ARTICLE_CODE;
        message->read_length = 32;
        break;

    case READ_DEVICE_SERIAL_NUMBER:
        message->command = I2C_READ_DEVICE_SERIAL_NUMBER;
        message->read_length = 32;
        break;

    case READ_AUTO_CLEANING:
        message->command = I2C_READ_WRITE_AUTO_CLEANING;
        message->read_length = 4;
        break;

    case WRITE_AUTO_CLEANING:
        message->command = I2C_READ_WRITE_AUTO_CLEANING;
        message->read_length = 0;
        message->length = 6;                         // Add the data length
        message->data[i++] = parameter >> 24 & 0xFF; // Split the parameter in bytes
        message->data[i++] = parameter >> 16 & 0xFF;
        message->data[i++] = I2C_calculate_CRC(message->data);
        message->data[i++] = parameter >> 8 & 0xFF;
        message->data[i++] = parameter & 0xFF;
        message->data[i++] = I2C_calculate_CRC(message->data + 3);
        break;

    default:
        return false;
    }

    return true;
}

uint8_t SPS30::I2C_calculate_CRC(uint8_t *data)
{
    uint8_t crc = I2C_CRC_INITIALIZATION;
    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ I2C_CRC_POLYNOMIAL;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

// SHDLC_send_command sends a command and reads back the response into a Message struct
bool SPS30::SHDLC_send_command(Message *response, uint8_t command, uint32_t parameter)
{
    if (command == READ_DATA_READY) // The SHDLC doesn't have a data ready command
    {
        return true;
    }

    SHDLC_create_command(response, command, parameter);

    _serial->flush(); // Flush anything pending on the serial port

    if (!SHDLC_send(response)) // Send the created command
    {
        return false;
    }

    delay(RX_DELAY_MS); // Give the SPS30 some time to respond

    if (!SHDLC_read(response)) // Read the response of the SPS30
    {
        return false;
    }

    if (response->state != 0) // Check the state response for errors
    {
        if (_SPS30_debug)
        {
            _debug->print(response->data[SHDLC_STATE_BYTE], HEX);
            _debug->println(" : state error");
        }
    }

    return true;
}

// serial_to_buffer reads the serial input into the buffer and performs byte unstuffing
bool SPS30::SHDLC_read(Message *response)
{
    uint8_t buffer[MAX_RECEIVE_BUFFER_LENGTH];

    uint32_t start_time = millis();
    bool byte_stuffing = false;
    bool header_received = false;

    uint8_t i = 0;
    uint8_t buffer_index = 0;

    while (!header_received) // Read the input stream until the last SHDLC_HEADER
    {
        if (millis() - start_time > TIME_OUT) // Prevent deadlock by timing out after a while
        {
            if (_SPS30_debug > 1)
            {
                _debug->print("TimeOut during reading byte ");
                _debug->println(i);
            }

            return false;
        }

        if (_serial->available())
        {
            buffer[i] = _serial->read();

            if (i == 0) // If it is the first byte
            {
                if (buffer[i] != SHDLC_HEADER) // Check if the received byte is the SHDLC header byte
                {

                    if (_SPS30_debug > 1)
                    {
                        _debug->print("Incorrect Header-> Expected 0x7E got ");
                        _debug->println(buffer[i]);
                    }

                    return 0;
                }
            }
            else // Else parse the bytes
            {
                if (buffer[i] == SHDLC_STUFFING_BYTE) // If the received byte is a stuffing byte
                {
                    i--;                  // Remove the stuffing byte
                    byte_stuffing = true; // And the next byte should be unstuffed
                }
                else if (byte_stuffing)
                {
                    buffer[i] = byte_unstuffing(buffer[i]);
                    byte_stuffing = false;
                }
                else if (buffer[i] == SHDLC_HEADER) // If a trailer byte is received stop the receiving
                {
                    buffer_index = i;

                    if (_SPS30_debug)
                    {
                        _debug->print("Received: ");
                        for (i = 0; i <= buffer_index; i++)
                        {
                            _debug->print(buffer[i], HEX);
                            _debug->print(" ");
                        }
                        _debug->print("length: ");
                        _debug->println(buffer_index);
                    }

                    /* if a board can not handle 115K you get uncontrolled input
                     * that can result in short /wrong messages
                     */
                    if (buffer_index < 3)
                    {
                        return false;
                    }

                    header_received = true;
                    break;
                }
            }

            i++;

            if (i > MAX_RECEIVE_BUFFER_LENGTH)
            {
                if (_SPS30_debug)
                {
                    _debug->println("Receive buffer full");
                }
                return false;
            }
        }
    }

    response->address = buffer[SHDLC_ADDRESS_BYTE];
    response->command = buffer[SHDLC_COMMAND_BYTE];
    response->state = buffer[SHDLC_STATE_BYTE];
    response->length = buffer[SHDLC_LENGTH_BYTE];

    for (int i = 0; i < response->length + 1; i++)
    {
        response->data[i] = buffer[SHDLC_DATA_BYTE + i];
    }

    uint8_t crc = SHDLC_calculate_CRC(response, true); // Check the CRC

    if (response->data[response->length] != crc)
    {
        if (_SPS30_debug)
        {
            _debug->print("CRC error-> expected ");
            _debug->print(response->data[response->length], HEX);
            _debug->print(", got ");
            _debug->println(crc, HEX);
        }
        return false;
    }

    return true;
}

// send_to_serial sends the _send_buffer over the set serial connection
bool SPS30::SHDLC_send(Message *message)
{
    if (_SPS30_debug)
    {
        _debug->print("Sending: ");
        _debug->print(SHDLC_HEADER, HEX);
        _debug->print(" ");
        _debug->print(message->address, HEX);
        _debug->print(" ");
        _debug->print(message->command, HEX);
        _debug->print(" ");
        _debug->print(message->length, HEX);
        _debug->print(" ");
        for (uint8_t i = 0; i < message->length + 1; i++)
        {
            _debug->print(message->data[i], HEX);
            _debug->print(" ");
        }
        _debug->print(SHDLC_HEADER, HEX);
        _debug->println("");
    }

    _serial->write(SHDLC_HEADER);
    _serial->write(message->address & 0x00FF);
    _serial->write(message->command & 0x00FF);
    _serial->write(message->length);

    for (uint8_t i = 0; i < message->length + 1; i++) // Transmit all the data + CRC
    {
        _serial->write(message->data[i]); // Send all the bytes over the set serial
    }

    _serial->write(SHDLC_HEADER);

    return true;
}

// SHDLC_create_command fills the buffer based on the given command and parameter and returns the length of
bool SPS30::SHDLC_create_command(Message *message, uint8_t command, uint32_t parameter)
{
    int i = 0;
    uint8_t tmp;

    message->address = 0;
    message->length = 0;

    switch (command)
    {
    case START_MEASUREMENT:
        message->command = SHDLC_START_MEASUREMENT;
        message->length = 2; // Add the data length
        message->data[i++] = 0x1;
        message->data[i++] = 0x3;
        break;

    case STOP_MEASUREMENT:
        message->command = SHDLC_STOP_MEASUREMENT;
        break;

    case READ_MEASURED_VALUE:
        message->command = SHDLC_READ_MEASURED_VALUE;
        break;

    case START_FAN_CLEANING:
        message->command = SHDLC_START_FAN_CLEANING;
        break;

    case RESET:
        message->command = SHDLC_RESET;
        break;

    case READ_DEVICE_PRODUCT_NAME:
        message->command = SHDLC_READ_DEVICE_INFO;
        message->length = 1;                                        // Add the data length
        message->data[i++] = SHDLC_READ_DEVICE_PRODUCT_NAME & 0x0F; // On these reads the command is used as data bytes
        break;

    case READ_DEVICE_ARTICLE_CODE:
        message->command = SHDLC_READ_DEVICE_INFO;
        message->length = 1;                                        // Add the data length
        message->data[i++] = SHDLC_READ_DEVICE_ARTICLE_CODE & 0x0F; // On these reads the command is used as data bytes
        break;

    case READ_DEVICE_SERIAL_NUMBER:
        message->command = SHDLC_READ_DEVICE_INFO;
        message->length = 1;                                         // Add the data length
        message->data[i++] = SHDLC_READ_DEVICE_SERIAL_NUMBER & 0x0F; // On these reads the command is used as data bytes
        break;

    case READ_AUTO_CLEANING:
        message->command = SHDLC_AUTO_CLEANING_INTERVAL;
        message->length = 1;    // Add the data length
        message->data[i++] = 0; // Add a subcommand, this value must be set to 0
        break;

    case WRITE_AUTO_CLEANING:
        message->command = SHDLC_AUTO_CLEANING_INTERVAL;
        message->length = 5;    // Add the data length
        message->data[i++] = 0; // Add a subcommand, this value must be set to 0

        tmp = parameter >> 24 & 0xFF; // Add the (byte stuffed) parameter
        i = byte_stuffing(message->data, tmp, i);
        tmp = parameter >> 16 & 0xFF;
        i = byte_stuffing(message->data, tmp, i);
        tmp = parameter >> 8 & 0xFF;
        i = byte_stuffing(message->data, tmp, i);
        tmp = parameter & 0xFF;
        i = byte_stuffing(message->data, tmp, i);
        break;

    default:
        return false;
    }

    // Add the CRC and check it for byte stuffing
    tmp = SHDLC_calculate_CRC(message, false);
    i = byte_stuffing(message->data, tmp, i);

    return true;
}

// SHDLC_calculate_CRC calculates the CRC, the CRC is calculated by taking the inverse of the LSB of the sum of all the bytes between the headers->
// First indicates the offset of the first byte to calculate from, and last the offset of the last byte->
uint8_t SPS30::SHDLC_calculate_CRC(Message *message, bool received)
{
    uint32_t crc = 0;

    crc += message->address;
    crc += message->command;
    crc += message->length;

    for (uint8_t i = 0; i < message->length; i++)
    {
        crc += message->data[i];
    }

    if (received)
    {
        crc += message->state;
    }

    return ~(crc & 0xff);
}

// byte_stuffing checks if a byte needs stuffing and stuffs it
uint8_t SPS30::byte_stuffing(uint8_t *buffer, uint8_t value, uint8_t offset)
{
    uint8_t x = 0;

    switch (value)
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
        buffer[offset++] = value;
    else
    {
        buffer[offset++] = 0x7D;
        buffer[offset++] = x;
    }

    return offset;
}

// byte_unstuffing unstuffs stuffed bytes
uint8_t SPS30::byte_unstuffing(uint8_t value)
{
    switch (value)
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
        if (_SPS30_debug)
        {
            _debug->print("Incorrect byte Unstuffing-> Got: ");
            _debug->println(value);
        }
        return NULL;
    }
}

// byte_to_float translates a byte array to a float
float SPS30::byte_to_float(uint8_t *buffer)
{
    uint32_t value;
    float float_value;

    for (byte i = 0; i < 4; i++)
    {
        value += (uint32_t)buffer[3 - i] << (i * 8);
    }

    memcpy(&float_value, &value, sizeof(float_value));

    return float_value;
}

// byte_to_U32 translates a byte array to an uint32_t
uint32_t SPS30::byte_to_U32(uint8_t *buffer)
{
    uint32_t value;

    for (byte i = 0; i < 4; i++)
    {
        value += (uint32_t)buffer[3 - i] << (i * 8);
    }

    return value;
}