/*!
* @Author Jason Hughes
* @Date October 2025
*
* @Brief Drive msg
*/

#pragma once

#include <cstdint>

namespace JackalSerial
{
struct Drive
{
    int8_t mode;
    float drive[2];

    Drive(int8_t m, float left, float right);
};
}
