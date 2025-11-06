/*!
* @Author: Jason Hughes
* @Date: November 2025
*
* @Brtief
*
*/


#include "jackal_serial/core/serializer.hpp"

using namespace JackalSerial;

std::vector<uint8_t> JackalSerializer::serializeTimeMsg(const uint32_t seconds, const uint32_t nanoseconds)
{
    std::vector<uint8_t> buffer(8);

    std::memcpy(&buffer[0], &seconds, 4);
    std::memcpy(&buffer[4], &nanoseconds, 4);

    return buffer;
}

std::vector<uint8_t> JackalSerializer::serializeDriveMsg(const Drive msg)
{
    std::vector<uint8_t> buffer(sizeof(int8_t) + 2 * sizeof(float));

    size_t offset = 0;
    std::memcpy(buffer.data() + offset, &msg.mode, sizeof(int8_t));
    offset += sizeof(int8_t);

    std::memcpy(buffer.data() + offset, &msg.drive[0], sizeof(float));
    offset += sizeof(float);

    std::memcpy(buffer.data() + offset, &msg.drive[1], sizeof(float));
    
    return buffer;
}
