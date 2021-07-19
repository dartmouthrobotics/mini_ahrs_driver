#include <string>
#include <serial/serial.h>
#include <mini_ahrs_driver/config.h>
#include <functional>
#include <iostream>
#include <thread>
#include <chrono>

#include <mini_ahrs_driver/MiniAHRSPacket.h>

namespace mini_ahrs_driver {

struct AHRSOrientationData {
    float roll_degrees;
    float pitch_degrees;
    float yaw_degrees;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float temperature;
}; // struct AHRSOrientationData

class MiniAHRSDriver
{
public:
    bool have_device_info_;
    bool run_polling_thread_;
    bool have_device_params_;
    int data_rate_hz_;
    int initial_alignment_time_seconds_;

    std::string device_serial_number_;
    std::string device_firmware_version_;

    MiniAHRSDriver(std::string serial_port_path, int baudrate);
    bool start();
    void stop();
    void setCallback(std::function<void(const AHRSOrientationData&)> cb);

    void AHRSDataPollingLoop();

    bool isRunning();

    ~MiniAHRSDriver();
    serial::Serial serial_connection_;
private:
    std::function<void(AHRSOrientationData)> data_callback_;
    std::thread worker_thread_;

    HeaderData parseHeader(const std::vector<uint8_t>& data);
    void handleGetDeviceInfoResponse(const std::vector<uint8_t>& message_body);
    void handleReadParamsCommandResponse(const std::vector<uint8_t>& message_body);
    AHRSOrientationData parseOrientationData(const std::vector<uint8_t>& data);

    void workerThreadMain();
}; // class MiniAHRSDriver

} // namespace mini_ahrs_driver
