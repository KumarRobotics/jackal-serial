/*!
* @Author: Jason Hughes
* @Date: October 2025
*
* @Brief: Format the outgoing packets 
* and decode the incoming packets
*/

#pragma once

#include <vector>
#include <cstdint>
#include <cstring>

namespace JackalSerial
{

enum class TopicID : uint16_t
{
    TIMESYNC = 0,
    DRIVE = 100,
    FEEDBACK = 110,
    STATUS = 111,
    NAVSATFIX = 112,
    IMUDATARAW = 113,
    IMUMAG = 114
};

class JackalPacket
{
    public:
        JackalPacket() = default;
        static std::vector<uint8_t> getSerialPacket(const TopicID topic_id, const std::vector<uint8_t> smsg);

    private:
        static uint8_t calculateChecksum(const std::vector<uint8_t>& data);
};

}
