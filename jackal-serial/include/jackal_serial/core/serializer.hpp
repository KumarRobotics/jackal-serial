/*!
* @Author: Jason Hughes
* @Date: November 2025
*
* @Brief: serialize and deserialize messages here
*/

#pragma once

#include <vector>
#include <cstdint>
#include <cstring>

#include "jackal_serial/core/drive.hpp"

namespace JackalSerial
{

struct JackalSerializer
{
    static std::vector<uint8_t> serializeTimeMsg(const uint32_t sec, const uint32_t nsec);
    static std::vector<uint8_t> serializeDriveMsg(const Drive msg);
};
}
