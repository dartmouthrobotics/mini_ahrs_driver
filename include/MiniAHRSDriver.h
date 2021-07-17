#include <string>

#include "config.h"
#include "serial/serial.h"

namespace mini_ahrs_driver
{

class MiniAHRSDriver
{

void Serial::serial serial_connection_

MiniAHRSDriver(std::string serial_port_path, int baudrate);

void start();

}; // class MiniAHRSDriver

} // namespace mini_ahrs_driver
