/*!
 * @Author Jason Hughes
 * @Date November 2025
 *
 * @Brief drive constructor
 *
 */

#include "jackal_serial/core/drive.hpp"

using namespace JackalSerial;

Drive::Drive(int8_t m, float left, float right) : mode(m), drive{left, right} { }
