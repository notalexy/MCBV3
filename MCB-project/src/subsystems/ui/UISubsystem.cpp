#include "UISubsystem.hpp"

namespace subsystems
{

UISubsystem::UISubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      refSerialTransmitter(drivers)
{
}

uint32_t UISubsystem::getUnusedGraphicName() {
    if (curGraphicName > 0xffffff) {
        //maybe signal some error? turn on the red led?
        return curGraphicName;
    } else {
        return curGraphicName++;
    }
}

uint8_t* UISubsystem::formatGraphicName(uint8_t array[3], uint32_t name) {
    //shifts and bitwise and
    array[0] = static_cast<uint8_t>((name >> 16) & 0xff);
    array[1] = static_cast<uint8_t>((name >> 8) & 0xff);
    array[2] = static_cast<uint8_t>(name & 0xff);
    return array;
}

void UISubsystem::initialize()
{
    
    this->restarting = false;

}

//guaranteed to be called
void UISubsystem::refresh()
{   
    run();

}

bool UISubsystem::run() {
    // The thread has exited the loop, meaning that there are no locked resources
    if (!this->isRunning())
    {
        // Restart the thread
        restart();
        // Reset the HUD elements
        // this->restartHud();
        curGraphicName = 0;
        restarting = false;
        needToDelete = true;
    }

    PT_BEGIN();

    PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

    //need to figure out delete at start. It seems like it wants to delete every time run is called
    // if(needToDelete){
    //     needToDelete = false; //probably this needs to be first to not go in this branch next time, so set it to false before the pt call
    //     PT_CALL(refSerialTransmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_ALL, 0));
    // }

    
    // If we try to restart the hud, break out of the loop
    while (!this->restarting)
    {
        
        
        
        // PT_CALL(refSerialTransmitter.sendGraphic(&textGraphic));
        // delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&textGraphic));
        // PT_WAIT_UNTIL(delayTimeout.execute());

        

        PT_YIELD();
    }
    // Breaking out of the loop successfully calls this method,
    // allowing us to know that all execution is over.
    PT_END();
}


}  // namespace subsystems