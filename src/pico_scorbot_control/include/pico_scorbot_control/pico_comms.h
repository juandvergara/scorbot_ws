#ifndef SCORBOT_PICO_COMMS_H
#define SCORBOT_PICO_COMMS_H

#include <libserial/SerialPort.h>
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

    PicoComms(const std::string &serial_master_device, const std::string &serial_slave_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_master_conn_(serial_master_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms)), 
        serial_slave_conn_(serial_slave_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {
    }

    void setup(const std::string &serial_master_device, const std::string &serial_slave_device, int32_t baud_rate, int32_t timeout_ms);
    void readJointValues(std::vector<double> &pos_joints);
    void setJointValues(std::vector<double> &target_joints);

    bool connected() const { return serial_master_conn_.isOpen() && serial_slave_conn_.isOpen(); }

    std::string sendMsg(const std::string &msg_to_send, bool print_output = false);
    std::vector<double> actual_pos_joints;
    std::vector<double> actual_vel_joints;

private:
    serial::Serial serial_master_conn_;
    serial::Serial serial_slave_conn_;///< Underlying serial connection
};

#endif // SCORBOT_PICO_COMMS_H