// #include "JetsonCommunication.hpp"
// #include <ctime>
// #include <cstring> 

<<<<<<< HEAD
// namespace communication
// {
//     JetsonCommunication::JetsonCommunication(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort port = tap::communication::serial::Uart::Uart1, bool isRxCRCEnforcementEnabled = true) :
//         DJISerial(drivers, port, isRxCRCEnforcementEnabled),
//         hasNewData(false)
//     {
//         // Good practice
//         memset(&lastCVData, 0, sizeof(lastCVData));
//         // Initial time
//         lastReceivedTime = getCurrentTime();
//     }

//     void JetsonCommunication::messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
//     {
//         // Serial message is 256 bytes at most. What are we tracking?
//         if (completeMessage.header.dataLength >= sizeof(CVData))
//         {
//             std::memcpy(&lastCVData, completeMessage.data, sizeof(CVData));
//             hasNewData = true;
//             lastReceivedTime = getCurrentTime();
//         }
//         else
//         {
//             // What kind of error handling when message fails?
//             hasNewData = false;
//         }
//     }
=======
namespace communication
{
    JetsonCommunication::JetsonCommunication(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort _port, bool isRxCRCEnforcementEnabled) :
        DJISerial(drivers, _port, isRxCRCEnforcementEnabled),
        port(_port),
        hasNewData(false)
    {
        // Good practice
        memset(&lastCVData, 0, sizeof(lastCVData));
        // Initial time
        lastReceivedTime = getCurrentTime();
    }

    void JetsonCommunication::messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
    {
        size_t bytesToCopy = completeMessage.header.dataLength;
        if (bytesToCopy > sizeof(CVData))
        {
            bytesToCopy = sizeof(CVData);
        }
        if (bytesToCopy > 0 && completeMessage.data != nullptr)
        {
            memcpy(&lastCVData, completeMessage.data, bytesToCopy);
            if (bytesToCopy < sizeof(CVData))
            {
                // need to do this for pointer arithmetic
                memset(reinterpret_cast<uint8_t*>(&lastCVData) + bytesToCopy, 0, sizeof(CVData) - bytesToCopy);
            }
            lastCVData.timestamp = getCurrentTime();
            hasNewData = true;
            lastReceivedTime = getCurrentTime();
        }
        else
        {
            hasNewData = false;
        }
    }
>>>>>>> fdb5fb34b678057dd1df5f762472988d8c535be9

//     // Will we constantly receive data in a stream?
//     void JetsonCommunication::update()
//     {
//         updateSerial();
    
//         if ((getCurrentTime() - lastReceivedTime) > CONNECTION_TIMEOUT)
//         {
//             hasNewData = false;
//         }
//     }

<<<<<<< HEAD
//     // Should I "consume" the CV data or continue to store it? (previous iteration consumed)
//     const CVData* JetsonCommunication::getLastCVData()
//     {
//         if (hasNewData)
//         {
//             hasNewData = false;
//             return &lastCVData;
//         }
//         else
//         {
//             return nullptr;
//         }
//     }

//     // Not really sure what this is doing and what the reinterpret cast is trying to do
//     bool JetsonCommunication::sendAutoAimOutput(const AutoAimOutput &output)
//     {
//         // Flexible ports?
//         tap::communication::serial::Uart::UartPort port = tap::communication::serial::Uart::Uart1;
//         const uint8_t* msgData = reinterpret_cast<const uint8_t*>(&output);
//         int bytesWritten = drivers->uart.write(port, msgData, sizeof(AutoAimOutput));
//         return (bytesWritten == sizeof(AutoAimOutput));
//     }
=======
    const CVData* JetsonCommunication::getLastCVData()
    {
        return hasNewData ? &lastCVData : nullptr;
    }

    bool JetsonCommunication::sendAutoAimOutput(AutoAimOutput &output)
    {
        // Flexible ports?
        tap::communication::serial::Uart::UartPort currentPort = port;
        // Update the timestamp before sending.
        output.timestamp = getCurrentTime();
        const uint8_t* msgData = reinterpret_cast<const uint8_t*>(&output);
        int bytesWritten = drivers->uart.write(currentPort, msgData, sizeof(AutoAimOutput));
        return (bytesWritten == sizeof(AutoAimOutput));
    }
>>>>>>> fdb5fb34b678057dd1df5f762472988d8c535be9

//     bool JetsonCommunication::isConnected() const
//     {
//         return ((getCurrentTime() - lastReceivedTime) <= CONNECTION_TIMEOUT);
//     }

//     void JetsonCommunication::clearNewDataFlag()
//     {
//         hasNewData = false;
//     }

<<<<<<< HEAD
//     uint64_t JetsonCommunication::getCurrentTime() const
//     {
//         std::time_t currentTime = std::time(nullptr);
//         return currentTime;
//     }
// }
=======
    uint64_t JetsonCommunication::getCurrentTime() const
    {
        std::time_t currentTime = std::time(nullptr);
        return currentTime;
    }

    tap::communication::serial::Uart::UartPort JetsonCommunication::getPort() const
    {
        return port;
    }
}
>>>>>>> fdb5fb34b678057dd1df5f762472988d8c535be9
