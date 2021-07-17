#include "MiniAHRSDriver.h"

namespace mini_ahrs_driver
{

MiniAHRSDriver::MiniAHRSDriver(std::string serial_port_path, int baudrate)
{
    serial_connection_ = serial::Serial(serial_port_path, baudrate, SERIAL_CONNECT_TIMEOUT_MS);
}

} // namespace mini_ahrs_driver
