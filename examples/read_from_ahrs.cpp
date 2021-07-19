#include <iostream>
#include <memory>

#include <mini_ahrs_driver/MiniAHRSDriver.h>

void print_usage() {
    std::cout << "Wrong number of arguments. The proper usage is: " << std::endl;
    std::cout << "./read_from_ahrs <serial_port> <baudrate>" << std::endl;
    std::cout << "Note that the baudrate must match what you configured using the windows UI!" << std::endl;
}

struct ahrs_callback {
    void operator()(mini_ahrs_driver::AHRSOrientationData data) {
        std::cout << std::dec << "Y " << data.yaw_degrees << " P " << data.pitch_degrees << " R " << data.roll_degrees << "    \r" << std::flush;
    }
};

int main(int argc, char** argv) {
    if (argc != 3) {
        print_usage();
        return 1;
    }

    std::string serial_port(argv[1]);
    int baudrate(std::stoi(argv[2]));

    std::unique_ptr<mini_ahrs_driver::MiniAHRSDriver> ahrs_driver;

    std::cout << "Creating serial port instance for path " << serial_port << " at baudrate " << baudrate << std::endl;
    try {
        ahrs_driver = std::unique_ptr<mini_ahrs_driver::MiniAHRSDriver>(new mini_ahrs_driver::MiniAHRSDriver(serial_port, baudrate));
    } catch(const serial::IOException& e) {
        std::cout << "Could not connect to serial port at " << serial_port << std::endl;
        std::cout << "error: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "    success." << std::endl;

    ahrs_callback cb;
    ahrs_driver->setCallback(
        std::function<void(mini_ahrs_driver::AHRSOrientationData)>(cb)
    );

    std::cout << "Starting MiniAHRS driver." << std::endl;
    bool success = ahrs_driver->start();
    if (!success) {
        ahrs_driver->stop();
    } else {
        while(1) {}
    }

    return 0;
}
