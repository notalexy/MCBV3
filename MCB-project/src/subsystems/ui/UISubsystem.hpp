#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "drivers.hpp"

namespace subsystems
{
using namespace tap::communication::serial;

class UISubsystem : public tap::control::Subsystem, ::modm::pt::Protothread, tap::communication::serial::RefSerialData
{

private:                                            // Private Variables
    tap::Drivers* drivers;
    RefSerialTransmitter refSerialTransmitter;

    uint32_t currGraphicName = 0;
    uint8_t graphicName[3];

    bool restarting = false; 
    bool needToDelete = true; 
    float fps = 0.0f;
    uint32_t startTime = 0;

    Tx::GraphicCharacterMessage textGraphic;
    char *bulletsRemainingText =(char*) malloc(5*sizeof(char));

    /** Width of the screen, in pixels. */
    static constexpr uint16_t SCREEN_WIDTH = 1920;
    /** Height of the screen, in pixels. */
    static constexpr uint16_t SCREEN_HEIGHT = 1080;

    int i = 0;

    tap::arch::MilliTimeout delayTimeout;
   

public:  // Public Methods
    UISubsystem(tap::Drivers* driver);
    ~UISubsystem() {}  // Intentionally blank


    /*
     * Call this function once, outside of the main loop.
     */
    void initialize();

    void refresh() override;

private:  // Private Methods
    bool run(); 
    void getUnusedGraphicName(uint8_t graphicName[3]);

};
}  // namespace subsystems