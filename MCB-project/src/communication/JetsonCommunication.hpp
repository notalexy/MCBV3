// #pragma once
// #include "tap/communication/serial/dji_serial.hpp"
// #include "tap/communication/serial/uart.hpp"

// #include "drivers.hpp"
// namespace communication
// {
// // Inconsistent across branches
// struct CVData 
// {
//     float x;
//     float y;
//     float z;
//     float confidence;
// };

// // What are we sending?
// struct AutoAimOutput
// {
//     uint8_t header = 0xA5;
//     uint16_t data_len = sizeof(float);
//     float pitch;
//     uint8_t newline = 0x0A;
// };

// class JetsonCommunication : public tap::communication::serial::DJISerial
// {
// public:
//     JetsonCommunication(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort port = tap::communication::serial::Uart::Uart1, bool isRxCRCEnforcementEnabled = true);
//     virtual ~JetsonCommunication() = default;
//     virtual void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) override;
//     void update();
//     const CVData* getLastCVData();
//     void clearNewDataFlag();
//     bool sendAutoAimOutput(const AutoAimOutput &output);
//     bool isConnected() const;
//     uint64_t getCurrentTime() const;
// private:
//     CVData lastCVData;
//     bool hasNewData;
//     uint64_t lastReceivedTime;

//     static constexpr uint32_t CONNECTION_TIMEOUT= 1000; // ms
// };
// } // namespace communication
