#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "drivers.hpp"
#include "util/ui/GraphicsContainer.hpp"

namespace subsystems
{
using namespace tap::communication::serial;

class UISubsystem : public tap::control::Subsystem, ::modm::pt::Protothread
{

public:
    static constexpr uint16_t SCREEN_WIDTH = 1920; //pixels
    static constexpr uint16_t SCREEN_HEIGHT = 1080; //pixels

private:  // Private Variables
    tap::Drivers* drivers;
    RefSerialTransmitter refSerialTransmitter;
    tap::arch::MilliTimeout delayTimeout; //for not sending things too fast and dropping packets

    static uint32_t currGraphicName;
    
    //for protothread
    bool restarting = false; 
    bool needToDelete = true; 
    static constexpr int TARGET_NUM_OBJECTS = 7;
    GraphicsObject* objectsToSend[TARGET_NUM_OBJECTS];
    int graphicsIndex=0;
    int innerGraphicsIndex=0;
    GraphicsObject* nextGraphicsObject=nullptr;
    RefSerialData::Tx::Graphic1Message message1;
    RefSerialData::Tx::Graphic2Message message2;
    RefSerialData::Tx::Graphic5Message message5;
    RefSerialData::Tx::Graphic7Message message7;
    bool wasteIsBetterFor3, wasteIsBetterFor4, wasteIsBetterFor6 = false;
    uint8_t wasteNameArray[3];

    //when get a/the command, it should set this
    GraphicsContainer* topLevelContainer = nullptr;


public:  // Public Methods
    UISubsystem(tap::Drivers* driver);
    ~UISubsystem() {}  // Intentionally blank


    /*
     * Call this function once, outside of the main loop.
     */
    void initialize();

    void refresh() override;

    static uint32_t getUnusedGraphicName();

    /*
     * Puts name into array (changing it in place), and returns array
     */
    static uint8_t* formatGraphicName(uint8_t array[3], uint32_t name);

    void setTopLevelContainer(GraphicsContainer* container){
        topLevelContainer = container;
    }

    
private:  // Private Methods
    bool run(); //for protothread

};
}  // namespace subsystems