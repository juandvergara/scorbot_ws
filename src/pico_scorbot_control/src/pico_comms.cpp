#include "pico_scorbot_control/pico_comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <vector>

void PicoComms::setup(const std::string &serial_master_device, const std::string &serial_slave_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);

    serial_master_conn_.setPort(serial_master_device);
    serial_master_conn_.setBaudrate(baud_rate);
    serial_master_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_master_conn_.open();

    serial_slave_conn_.setPort(serial_slave_device);
    serial_slave_conn_.setBaudrate(baud_rate);
    serial_slave_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_slave_conn_.open();
}

void PicoComms::readJointValues(std::vector<double> &data_joints)
{
    std::string response = sendMsg("e\n");
    std::string delimiter = ",";

    while (response.find(delimiter) != std::string::npos)
    {
        std::string dato = response.substr(0, response.find(','));
        actual_pos_joints.push_back(std::atof(dato.c_str()));
        response.erase(0, response.find(',') + 1);
    }

    for (size_t i = 0; i < actual_pos_joints.size(); i++)
    {
        data_joints[i] = actual_pos_joints[i];
    }

    response = sendMsg("a\n");

    while (response.find(delimiter) != std::string::npos)
    {
        std::string dato = response.substr(0, response.find(','));
        actual_vel_joints.push_back(std::atof(dato.c_str()));
        response.erase(0, response.find(',') + 1);
    }
    for (size_t i = 0; i < actual_pos_joints.size(); i++)
    {
        data_joints[i + actual_pos_joints.size()] = actual_vel_joints[i];
    }
}

void PicoComms::setJointValues(std::vector<double> &target_joints)
{
    std::stringstream ss;
    ss << "p ";
    for (size_t i = 0; i < target_joints.size(); i++)
    {
        ss << target_joints[i];
        if (i < target_joints.size() - 1)
            ss << ',';
    }
    ss << "\n";
    sendMsg(ss.str(), false);
}

std::string PicoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_master_conn_.write(msg_to_send);
    std::string response_master = serial_master_conn_.readline();

    serial_slave_conn_.write(msg_to_send);
    std::string response_slave = serial_slave_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response_master + response_slave;
}