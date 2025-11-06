/*!
* @Author: Jason Hughes
* @Date: October 2025
*
* @Brief: A wrapper around the serial connection
* to the jackal
*/

#pragma once

#include <utility>
#include <string>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "jackal_serial/core/serializer.hpp"
#include "jackal_serial/core/packets.hpp"

namespace JackalSerial
{
class SerialCore
{
    public:
        SerialCore() = default;
        SerialCore(std::string& dev, int baud);

        void writeTime(std::pair<uint32_t, uint32_t> time);
	void writeDriveMsg(Drive drive);
    private:
        LibSerial::SerialPort serial_port_;
};
}
