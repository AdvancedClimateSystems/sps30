/************************************************************************************
 *  Copyright (c) January 2019, version 1.0     Paul van Haastrecht
 *
 *  Version 1.0 Yorick Smilda
 *  - rewrite from the version of Paulvha.
 *
 *  =========================  Highlevel description ================================
 *
 *  In this invidual reading example you can select which data AND in which order you
 *  want the data to be displayed.
 *
 *  ================================ Disclaimer ======================================
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  ===================================================================================
 *
 *  NO support, delivered as is, have fun, good luck !!
 */

#include "sps30.h"

// Define debug level, make sure to change the serial interface in the printf.h to the one you want to use for debugging
// 0 : no messages.
// 1 : request sending and receiving.
// 2 : request sending and receiving + show protocol errors.
#define DEBUG 0

// Define the serial port you want to use for debugging information and the serial port you want to use for the SPS30.
#define _SERIAL Serial
#define _SPS30 Serial1

// Change the sensor read out frequency.
#define READ_DELAY 3000

// Edit the array with the values you want to readout.
// Any read value can be added, even multiple times.
// Input options are defined in the readme.
uint8_t values[] = {MassPM1, NumPM1, MassPM2, NumPM2};
uint8_t value_size = sizeof(values) / sizeof(values[0]);

//
// You don't have to change anything below this
//

// Function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serial_trigger(char *mess);
void error_log(char *mess);
void get_device_info();
void print_device_info(char *mess, char *buf, uint8_t ret);
bool read_sensor_data();

SPS30_UART sps30;

void setup()
{
  _SERIAL.begin(115200);

  while (!_SERIAL)
    ;

  serial_trigger("SPS30-Example1: Basic reading. press <enter> to start");
  _SERIAL.println(F("Trying to connect"));

  sps30.enable_debugging(DEBUG); // Set the debug level.

  _SPS30.begin(115200);              // Start the Serial port you want to use at a baudrate of 115200bps.
  if (sps30.begin(&_SPS30) == false) // Pass the serial port along to the library and check if the SPS30 is available.
  {
    error_log("The SPS30 is not responding/available.");
  }

  if (sps30.reset() == false) // Reset the SPS30
  {
    error_log("Could not reset the sensor.");
  }

  get_device_info();

  if (sps30.start() == true)
  {
    _SERIAL.println(F("Measurement started!"));
  }
  else
  {
    error_log("Could not start the measurement");
  }

  serial_trigger("Hit <enter> to continue reading!");
}

void loop()
{
  read_sensor_data();
  delay(READ_DELAY);
}

// get_device_info prints out all the device information.
void get_device_info()
{
  char buf[32];
  uint8_t ret;

  ret = sps30.get_serial_number(buf, 32); // Read the serial number
  print_device_info("Serial number", buf, ret);

  ret = sps30.get_product_name(buf, 32); // Read the product name
  print_device_info("Product name", buf, ret);

  ret = sps30.get_article_code(buf, 32); // Read the article code
  print_device_info("Article code", buf, ret);
}

// print_device_info prints a string based on the ret value and the provided message
void print_device_info(char *mess, char *buf, uint8_t ret)
{
  if (ret == ERR_OK)
  {
    _SERIAL.print(mess);
    _SERIAL.print(" : ");

    if (strlen(buf) > 0)
    {
      _SERIAL.println(buf);
    }
    else
    {
      _SERIAL.println(F("not available"));
    }
  }
  else
  {
    _SERIAL.print("could not get the");
    _SERIAL.print(mess);
    _SERIAL.println(".");
  }
}

bool read_sensor_data()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  if (header) // First run create the header.
  {
    for (uint8_t i = 0; i < value_size; i++)
    {
      switch (values[i])
      {
      case MassPM1:
        _SERIAL.print(F("MassPM1\t"));
        break;
      case MassPM2:
        _SERIAL.print(F("MassPM2\t"));
        break;
      case MassPM4:
        _SERIAL.print(F("MassPM4\t"));
        break;
      case MassPM10:
        _SERIAL.print(F("MassPM10\t"));
        break;
      case NumPM0:
        _SERIAL.print(F("NumPM0\t"));
        break;
      case NumPM1:
        _SERIAL.print(F("NumPM1\t"));
        break;
      case NumPM2:
        _SERIAL.print(F("NumPM2\t"));
        break;
      case NumPM4:
        _SERIAL.print(F("NumPM4\t"));
        break;
      case NumPM10:
        _SERIAL.print(F("NumPM10\t"));
        break;
      case PartSize:
        _SERIAL.print(F("Prtsize\t"));
        break;
      }
    }

    header = false;
  }

  for (uint8_t i = 0; i < value_size; i++)
  {
    switch (values[i])
    {
    case MassPM1:
      _SERIAL.print(sps30.get_mass_PM1());
      _SERIAL.print(F("\t"));
      break;
    case MassPM2:
      _SERIAL.print(sps30.get_mass_PM2());
      _SERIAL.print(F("\t"));
      break;
    case MassPM4:
      _SERIAL.print(sps30.get_mass_PM4());
      _SERIAL.print(F("\t"));
      break;
    case MassPM10:
      _SERIAL.print(sps30.get_mass_PM10());
      _SERIAL.print(F("\t"));
      break;
    case NumPM0:
      _SERIAL.print(sps30.get_num_PM0());
      _SERIAL.print(F("\t"));
      break;
    case NumPM1:
      _SERIAL.print(sps30.get_num_PM1());
      _SERIAL.print(F("\t"));
      break;
    case NumPM2:
      _SERIAL.print(sps30.get_num_PM2());
      _SERIAL.print(F("\t"));
      break;
    case NumPM4:
      _SERIAL.print(sps30.get_num_PM4());
      _SERIAL.print(F("\t"));
      break;
    case NumPM10:
      _SERIAL.print(sps30.get_num_PM10());
      _SERIAL.print(F("\t"));
      break;
    case PartSize:
      _SERIAL.print(sps30.get_part_size());
      _SERIAL.print(F("\t"));
      break;
    }
  }
  _SERIAL.print(F("\n"));
}

void error_log(char *mess)
{
  _SERIAL.println(mess);
  _SERIAL.println("Restarting program!");
  setup();
}

void serial_trigger(char *mess)
{
  _SERIAL.println();
  _SERIAL.println(mess);

  while (!_SERIAL.available())
  {
    // Wait till something is sent over the serial line
  }

  while (_SERIAL.available())
  {
    _SERIAL.read(); // Empty the read buffer
  }
}
