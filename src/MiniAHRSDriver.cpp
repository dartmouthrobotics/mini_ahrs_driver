#include "mini_ahrs_driver/MiniAHRSDriver.h"
#include "mini_ahrs_driver/MiniAHRSPacket.h"

#include <cstdint>
#include <iomanip>

namespace mini_ahrs_driver
{

MiniAHRSDriver::MiniAHRSDriver(std::string serial_port_path, int baudrate, double KA, double KG) : serial_connection_(serial_port_path, baudrate, serial::Timeout::simpleTimeout(SERIAL_CONNECT_TIMEOUT_MS)), run_polling_thread_(false), have_device_params_(false), KA_(KA), KG_(KG)
{
    if (!serial_connection_.isOpen()) {
        throw std::runtime_error(std::string("Could not open serial port at path ") + serial_port_path);
    }
}

bool MiniAHRSDriver::isRunning() {
    return worker_thread_.joinable();
}

void MiniAHRSDriver::stop()
{
    if (isRunning()) {
        run_polling_thread_ = false;
        worker_thread_.join();

        StopDeviceCommandPacket packet;
        serial_connection_.write(packet.buffer);
    }
}

MiniAHRSDriver::~MiniAHRSDriver()
{
    stop();
    serial_connection_.close();
}

void MiniAHRSDriver::setCallback(std::function<void(const AHRSOrientationData&)> cb) {
    data_callback_ = cb;
}

bool MiniAHRSDriver::start() {
    StopDeviceCommandPacket stop_command;
    serial_connection_.write(stop_command.buffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    serial_connection_.read(serial_connection_.available()); // clear the buf

    run_polling_thread_ = true;
    GetDeviceInfoCommandPacket get_device_info_command;

    serial_connection_.write(get_device_info_command.buffer);
    worker_thread_ = std::thread(&MiniAHRSDriver::workerThreadMain, this); 
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    if (!have_device_info_) {
        run_polling_thread_ = false;
        worker_thread_.join();

        return false;
    }

    ReadMiniAHRSParamsCommandPacket read_params_packet;
    serial_connection_.write(read_params_packet.buffer); 

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (!have_device_params_) {
        run_polling_thread_ = false;
        worker_thread_.join();

        return false;
    }

    StartOrientationDataStreamCommandPacket start_command;
    serial_connection_.write(start_command.buffer);

    std::this_thread::sleep_for(
        std::chrono::seconds(initial_alignment_time_seconds_)
    );

    return true;
}


HeaderData MiniAHRSDriver::parseHeader(const std::vector<uint8_t>& data) {
    if (data[0] != 0xAA || data[1] != 0x55) {
        throw std::runtime_error("Malformed header!");
    }

    HeaderData result;
    result.message_type = data[2];
    result.data_identifier = data[3];

    uint8_t message_size_raw[2];
    message_size_raw[0] = data[4];
    message_size_raw[1] = data[5];

    uint16_t message_length;
    std::memcpy(&message_length, message_size_raw, sizeof message_length);

    result.message_length = message_length;

    return result;
}

AHRSOrientationData MiniAHRSDriver::parseOrientationData(const std::vector<uint8_t>& message_body) {
    AHRSOrientationData result;

    uint8_t yaw_raw[2];
    uint8_t pitch_raw[2];
    uint8_t roll_raw[2];

    uint8_t gyro_x_raw[2];
    uint8_t gyro_y_raw[2];
    uint8_t gyro_z_raw[2];

    uint8_t accel_x_raw[2];
    uint8_t accel_y_raw[2];
    uint8_t accel_z_raw[2];

    uint8_t mag_x_raw[2];
    uint8_t mag_y_raw[2];
    uint8_t mag_z_raw[2];

    uint8_t temp_raw[2];

    uint16_t yaw;
    int16_t pitch;
    int16_t roll;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    int16_t temperature;

    yaw_raw[0] = message_body[0];
    yaw_raw[1] = message_body[1];

    pitch_raw[0] = message_body[2];
    pitch_raw[1] = message_body[3];

    roll_raw[0] = message_body[4];
    roll_raw[1] = message_body[5];

    gyro_x_raw[0] = message_body[6];
    gyro_x_raw[1] = message_body[7];

    gyro_y_raw[0] = message_body[8];
    gyro_y_raw[1] = message_body[9];

    gyro_z_raw[0] = message_body[10];
    gyro_z_raw[1] = message_body[11];

    accel_x_raw[0] = message_body[12];
    accel_x_raw[1] = message_body[13];

    accel_y_raw[0] = message_body[14];
    accel_y_raw[1] = message_body[15];

    accel_z_raw[0] = message_body[16];
    accel_z_raw[1] = message_body[17];

    mag_x_raw[0] = message_body[18];
    mag_x_raw[1] = message_body[19];

    mag_y_raw[0] = message_body[20];
    mag_y_raw[1] = message_body[21];

    mag_z_raw[0] = message_body[22];
    mag_z_raw[1] = message_body[23];

    temp_raw[0] = message_body[32];
    temp_raw[1] = message_body[33];

    std::memcpy(&yaw, &yaw_raw, sizeof yaw);
    std::memcpy(&pitch, &pitch_raw, sizeof pitch);
    std::memcpy(&roll, &roll_raw, sizeof roll);

    std::memcpy(&gyro_x, &gyro_x_raw, sizeof gyro_x);
    std::memcpy(&gyro_y, &gyro_y_raw, sizeof gyro_y);
    std::memcpy(&gyro_z, &gyro_z_raw, sizeof gyro_z);

    std::memcpy(&mag_x, &mag_x_raw, sizeof mag_x);
    std::memcpy(&mag_y, &mag_y_raw, sizeof mag_y);
    std::memcpy(&mag_z, &mag_z_raw, sizeof mag_z);

    std::memcpy(&acc_x, &accel_x_raw, sizeof acc_x);
    std::memcpy(&acc_y, &accel_y_raw, sizeof acc_y);
    std::memcpy(&acc_z, &accel_z_raw, sizeof acc_z);

    std::memcpy(&temperature, &temp_raw, sizeof temperature);

    result.acc_x = double(acc_x) / KA_;
    result.acc_y = double(acc_y) / KA_;
    result.acc_z = double(acc_z) / KA_;

    result.gyro_x = double(gyro_x) / KG_;
    result.gyro_y = double(gyro_y) / KG_;
    result.gyro_z = double(gyro_z) / KG_;

    double nano_tesla_to_tesla = 1.0 / 1.0e-9; 
    result.mag_x = double(mag_x) * 10.0;
    result.mag_y = double(mag_y) * 10.0;
    result.mag_z = double(mag_z) * 10.0;

    result.yaw_degrees = double(yaw) / 100.0;
    result.pitch_degrees = double(pitch) / 100.0;
    result.roll_degrees = double(roll) / 100.0;

    result.temperature = double(temperature) / 10.0;

    return result;
}

void MiniAHRSDriver::handleGetDeviceInfoResponse(const std::vector<uint8_t>& message_body) {
    if (have_device_info_) {
        std::cerr << "Already have device info. Skipping." << std::endl;
        return;
    }

    std::string serial_number("");
    for (int i = 0; i < 8; ++i) {
        serial_number.push_back(message_body[i]);
    }

    std::string firmware_version("");
    for (int i = 8; i < 48; ++i) {
        firmware_version.push_back(message_body[i]);
    }

    have_device_info_ = true;
    device_serial_number_ = serial_number;
    device_firmware_version_ = firmware_version;
}

void MiniAHRSDriver::workerThreadMain() {
    try {
        while (run_polling_thread_) {
            AHRSDataPollingLoop();
        }
    } catch (const std::runtime_error e) {
        StopDeviceCommandPacket packet;
        serial_connection_.write(packet.buffer);
        return;
    }
}

void MiniAHRSDriver::handleReadParamsCommandResponse(const std::vector<uint8_t>& message_body)
{
    uint8_t data_rate_raw[2];
    data_rate_raw[0] = message_body[0];
    data_rate_raw[1] = message_body[1];

    uint8_t initial_alignment_time_raw[2];
    initial_alignment_time_raw[0] = message_body[2];
    initial_alignment_time_raw[1] = message_body[3];

    uint16_t data_rate;
    uint16_t alignment_time;

    std::memcpy(
        &data_rate,
        data_rate_raw,
        sizeof data_rate
    );

    std::memcpy(
        &alignment_time,
        initial_alignment_time_raw,
        sizeof alignment_time
    );

    initial_alignment_time_seconds_ = alignment_time;
    data_rate_hz_ = data_rate;
    have_device_params_ = true;
}

void MiniAHRSDriver::AHRSDataPollingLoop() {
    // read the first five bytes
    if (serial_connection_.available() < HEADER_LENGTH_BYTES) {
        return;
    }

    // look for the first two bytes of a message
    std::vector<uint8_t> message_header;
    serial_connection_.read(message_header, HEADER_LENGTH_BYTES);

    HeaderData parsed_header;
    try {
        parsed_header = parseHeader(message_header);
    } catch (std::runtime_error e) {
        //std::cout << "Invalid header!" << std::endl;
        //for (auto& byte : message_header) {
        //    std::cout << std::hex << int(byte) << " ";
        //}
        //std::cout << std::endl;
        throw std::runtime_error("Invalid header received. Cannot continue parsing!");
    }

    std::vector<uint8_t> message_body;
    serial_connection_.read(message_body, parsed_header.message_length - 4);

    switch (parsed_header.data_identifier) {
        case 0x12: {
            handleGetDeviceInfoResponse(message_body);
            break;
        }
        case 0x33: {
            AHRSOrientationData data = parseOrientationData(message_body);

            if (data_callback_) {
                data_callback_(data);
            }

            break;
        }
        case 0x41: {
            handleReadParamsCommandResponse(message_body);
            break;
        }
        default: {
            std::cout << "Unrecognized command format!" << std::endl;
            break;
        }
    }
}

} // mini_ahrs_driver
