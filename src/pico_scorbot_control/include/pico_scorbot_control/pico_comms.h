#ifndef SCORBOT_PICO_COMMS_H
#define SCORBOT_PICO_COMMS_H

#include <serial/serial.h>
#include <cstring>
#include <sstream>
#include <cstdlib>
#include <vector>

class PicoComms
{

public:
    PicoComms()
    {
    }

    PicoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {
    }

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void sendEmptyMsg();
    void readEncoderValues(int &val_1, int &val_2);
    void setMotorValues(int val_1, int val_2);

    bool connected() const { return serial_conn_.isOpen(); }

    std::string sendMsg(const std::string &msg_to_send, bool print_output = false);
    std::vector<float> target_joints;

private:
    serial::Serial serial_conn_; ///< Underlying serial connection
};

#endif // SCORBOT_PICO_COMMS_H