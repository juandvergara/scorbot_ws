#include "pico_scorbot_control/pico_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <vector>

void PicoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
}

void PicoComms::readJointValues(std::vector<float> &pos_joints)
{
    std::string response = sendMsg("e\r");
    std::string delimiter = ",";

    while (response.find(delimiter) != std::string::npos) {
        std::string dato = response.substr(0, response.find(','));
        actual_pos_joints.push_back(std::atof(dato.c_str()));
        response.erase(0, response.find(',') + 1);
    }

    for (size_t i = 0; i < actual_pos_joints.size(); i++)
    {
        pos_joints[i] = actual_pos_joints[i]; 
    }
}

void PicoComms::setJointValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}


std::string PicoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline(); 

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}