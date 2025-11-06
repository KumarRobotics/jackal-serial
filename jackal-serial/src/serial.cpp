
/*!
* @Author: Jason Hughes
* @Date: October 2025
*
* @Brief: A wrapper around the serial connection
* to the jackal
*/
#include <iostream>
#include "jackal_serial/core/serial.hpp"

using namespace JackalSerial;

SerialCore::SerialCore(std::string& dev, int baud)
{
    serial_port_.Open(dev);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    std::cout << "[SERIAL] Connected to Jackal" << std::endl;
}   


void SerialCore::writeTime(std::pair<uint32_t, uint32_t> time)
{
    std::cout << "[SERIAL] Writing Time Sync Packet" << std::endl;
    std::vector<uint8_t> serialized_msg = JackalSerializer::serializeTimeMsg(time.first, time.second);
    std::vector<uint8_t> packet = JackalPacket::getSerialPacket(TopicID::TIMESYNC, serialized_msg);

    serial_port_.Write(packet);

    //std::vector<uint8_t> response;
    //serial_port_.Read(response, 4096, 100);
}

void SerialCore::writeDriveMsg(Drive drive)
{
    std::vector<uint8_t> serialized_msg = JackalSerializer::serializeDriveMsg(drive);
    std::vector<uint8_t> packet = JackalPacket::getSerialPacket(TopicID::DRIVE, serialized_msg);

    serial_port_.Write(packet);
    
    //std::vector<uint8_t> response;
    //serial_port_.Read(response, 4096, 100);
}
