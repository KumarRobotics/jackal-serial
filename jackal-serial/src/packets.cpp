/*!
* @Author: Jason Hughes
* @Date: October 2025 
*
* @Brief: create the packets to 
* send over serial
*
*/
#include "jackal_serial/core/packets.hpp"

using namespace JackalSerial;

std::vector<uint8_t> JackalPacket::getSerialPacket(const TopicID topic_id, const std::vector<uint8_t> serialized_msg)
{
    std::vector<uint8_t> packet;
    packet.push_back(0xFF);
    packet.push_back(0xFE);

    uint16_t msg_len = serialized_msg.size();
    packet.push_back(msg_len & 0xFF);
    packet.push_back((msg_len >> 8) & 0xFF);

    std::vector<uint8_t> len_bytes = {packet[2], packet[3]};
    packet.push_back(calculateChecksum(len_bytes));

    uint16_t topic_id_int = static_cast<uint16_t>(topic_id);
    packet.push_back(topic_id_int & 0xFF);
    packet.push_back((topic_id_int >> 8) & 0xFF);
    
    packet.insert(packet.end(), serialized_msg.begin(), serialized_msg.end());

    std::vector<uint8_t> data_for_checksum = {packet[5], packet[6]};
    data_for_checksum.insert(data_for_checksum.end(), serialized_msg.begin(), serialized_msg.end());
    packet.push_back(calculateChecksum(data_for_checksum));

    return packet;
}

uint8_t JackalPacket::calculateChecksum(const std::vector<uint8_t>& data)
{
    uint32_t sum = 0;
    for (uint8_t rawbyte : data) {
        sum += rawbyte;
    }
    return 255 - (sum % 256);
}


