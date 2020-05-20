/************************************************************************************
    Copyright (c) May 2020, version 1.0     Yorick Smilda

    ================================ Disclaimer ======================================
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    ===================================================================================

    NO support, delivered as is, have fun, good luck !!
*/

#include "sps30.h"

// Enable the debugging mode
#define DEBUG true

// Define the serial port you want to use for debugging information and the serial port you want to use for the SPS30
#define _SERIAL Serial
#define _SPS30 Serial1

// Change the sensor read out frequency
#define READ_DELAY 3000

//
// You don't have to change anything below this
//

// Function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void error_log(char *mess);
void get_device_info();
void print_device_info(char *mess, char *buf, boolean succeeded);
void get_device_status();
boolean read_sensor_data();

SPS30 sps30;

void setup()
{
    _SERIAL.begin(115200);

    while (!_SERIAL) // Wait for the serial monitor to connect to the debug serial.
        ;

    _SERIAL.println(F("Trying to connect"));

    if (DEBUG == true)
    {
        sps30.enable_debugging(&_SERIAL); // Enable debugging.
    }

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

    get_device_status();
}

void loop()
{
    read_sensor_data();

    sps30.sleep(); // Put the SPS30 to sleep between measurements to save some energy.

    delay(READ_DELAY);

    sps30.wake_up(); // Wake up the SPS30 before reading the measurements.
}

// get_device_info prints out all the device information.
void get_device_info()
{
    char buf[32];
    boolean succeeded;

    succeeded = sps30.get_serial_number(buf, 32); // Read the serial number
    print_device_info("Serial number", buf, succeeded);

    succeeded = sps30.get_product_type(buf, 32); // Read the product name
    print_device_info("Product type", buf, succeeded);
}

// print_device_info prints a string based on the ret value and the provided message
void print_device_info(char *mess, char *buf, boolean succeeded)
{
    if (succeeded == true)
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
        _SERIAL.println("");
    }
    else
    {
        _SERIAL.print("could not get the ");
        _SERIAL.print(mess);
        _SERIAL.println(".");
        _SERIAL.println("");
    }
}

void get_device_status()
{
    delay(3000); // Wait for all the statusses to become available.

    boolean error;

    sps30.read_speed_status(&error);

    if (error)
    {
        _SERIAL.println("Fan speed is too low or to high.");
    }
    else
    {
        _SERIAL.println("Fan speed is ok.");
    }

    sps30.read_fan_status(&error);

    if (error)
    {
        _SERIAL.println("Fan is switched on, but the measured fan speed is 0 RPM.");
    }
    else
    {
        _SERIAL.println("Fan works as expected.");
    }

    sps30.read_laser_status(&error);

    if (error)
    {
        _SERIAL.println("Laser current is ok.");
    }
    else
    {
        _SERIAL.println("Laser is switched on and current is out of range.");
    }
}

// read_sensor_data reads the sensor data
boolean read_sensor_data()
{
    static boolean header = true;
    Measurements val;

    while (sps30.get_values(&val) == false)
    {
        delay(1000); // Try every second to get new values.
    }

    if (header) // Only print the header the first time.
    {
        _SERIAL.println(F("-------------Mass -----------    ------------- Number --------------   -Average-"));
        _SERIAL.println(F("     Concentration [μg/m3]             Concentration [#/cm3]             [μm]"));
        _SERIAL.println(F("P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n"));
        header = false;
    }

    _SERIAL.print(val.MassPM1);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.MassPM2);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.MassPM4);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.MassPM10);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.NumPM0);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.NumPM1);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.NumPM2);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.NumPM4);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.NumPM10);
    _SERIAL.print(F("\t"));
    _SERIAL.print(val.PartSize);
    _SERIAL.print(F("\n"));
}

void error_log(char *mess)
{
    _SERIAL.println(mess);
    _SERIAL.println("Restarting program!");
    setup();
}