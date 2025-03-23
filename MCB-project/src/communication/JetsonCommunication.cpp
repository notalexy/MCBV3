#include "JetsonCommunication.hpp"
#include <ctime>
#include <cstring> 

namespace communication
{
    JetsonCommunication::JetsonCommunication(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort port = tap::communication::serial::Uart::Uart1, bool isRxCRCEnforcementEnabled = true) :
        DJISerial(drivers, port, isRxCRCEnforcementEnabled),
        hasNewData(false)
    {
        // Good practice
        memset(&lastCVData, 0, sizeof(lastCVData));
        // Initial time
        lastReceivedTime = getCurrentTime();
    }

    void JetsonCommunication::messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
    {
        // Serial message is 256 bytes at most. What are we tracking?
        if (completeMessage.header.dataLength >= sizeof(CVData))
        {
            std::memcpy(&lastCVData, completeMessage.data, sizeof(CVData));
            hasNewData = true;
            lastReceivedTime = getCurrentTime();
        }
        else
        {
            // What kind of error handling when message fails?
            hasNewData = false;
        }
    }

    // Will we constantly receive data in a stream?
    void JetsonCommunication::update()
    {
        updateSerial();
    
        if ((getCurrentTime() - lastReceivedTime) > CONNECTION_TIMEOUT)
        {
            hasNewData = false;
        }
    }

    // Should I "consume" the CV data or continue to store it? (previous iteration consumed)
    const CVData* JetsonCommunication::getLastCVData()
    {
        if (hasNewData)
        {
            hasNewData = false;
            return &lastCVData;
        }
        else
        {
            return nullptr;
        }
    }

    // Not really sure what this is doing and what the reinterpret cast is trying to do
    bool JetsonCommunication::sendAutoAimOutput(const AutoAimOutput &output)
    {
        // Flexible ports?
        tap::communication::serial::Uart::UartPort port = tap::communication::serial::Uart::Uart1;
        const uint8_t* msgData = reinterpret_cast<const uint8_t*>(&output);
        int bytesWritten = drivers->uart.write(port, msgData, sizeof(AutoAimOutput));
        return (bytesWritten == sizeof(AutoAimOutput));
    }

    bool JetsonCommunication::isConnected() const
    {
        return ((getCurrentTime() - lastReceivedTime) <= CONNECTION_TIMEOUT);
    }

    void JetsonCommunication::clearNewDataFlag()
    {
        hasNewData = false;
    }

    uint64_t JetsonCommunication::getCurrentTime() const
    {
        std::time_t currentTime = std::time(nullptr);
        return currentTime;
    }
}