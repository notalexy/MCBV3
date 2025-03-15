#include "UISubsystem.hpp"

namespace subsystems
{

UISubsystem::UISubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      refSerialTransmitter(drivers)
{
}

void UISubsystem::getUnusedGraphicName(uint8_t graphicName[3])
{
    if (currGraphicName > 0xffffff)
    {
        return;
    }
    else
    {
        graphicName[0] = static_cast<uint8_t>((currGraphicName >> 16) & 0xff);
        graphicName[1] = static_cast<uint8_t>((currGraphicName >> 8) & 0xff);
        graphicName[2] = static_cast<uint8_t>(currGraphicName & 0xff);
        currGraphicName++;
    }
}

void UISubsystem::initialize()
{
    bulletsRemainingText[0]='t';
    bulletsRemainingText[1]='e';
    bulletsRemainingText[2]='s';
    bulletsRemainingText[3]='t';
    bulletsRemainingText[4]='\0';

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &textGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        0, //graphic layer
        Tx::GraphicColor::ORANGE);

    RefSerialTransmitter::configCharacterMsg(40, 4, SCREEN_WIDTH / 2 - 150, 200, bulletsRemainingText, &textGraphic);

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
        currGraphicName = 0;
        restarting = false;
        needToDelete = true;
    }

    PT_BEGIN();

    PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());
    // if(needToDelete){
    //     PT_CALL(refSerialTransmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_ALL, 0));
    //     needToDelete = false;
    // }

    
    // If we try to restart the hud, break out of the loop
    while (!this->restarting)
    {
        startTime = tap::arch::clock::getTimeMicroseconds();
    //     // for (index = 0; index < numIndicators; index++)
    //     // {
    //         // PT_CALL(hudIndicators[index]->update());
            
    //     // }

    //     // Calculate the time it took to update the HUD
        
        
        
        PT_CALL(refSerialTransmitter.sendGraphic(&textGraphic));
        delayTimeout.restart(Tx::getWaitTimeAfterGraphicSendMs(&textGraphic));
        PT_WAIT_UNTIL(delayTimeout.execute());

        textGraphic.graphicData.operation = Tx::GRAPHIC_MODIFY;
        
        i=!i;
            
        fps = 1e6 / (tap::arch::clock::getTimeMicroseconds() - startTime);
        bulletsRemainingText[0]=(static_cast<int>(fps/100)%10) +'0';
        bulletsRemainingText[1]=(static_cast<int>(fps/10)%10) +'0';
        bulletsRemainingText[2]=(static_cast<int>(fps)%10) +'0';
        bulletsRemainingText[3]=i?'.':',';
        bulletsRemainingText[4]='\0';
        RefSerialTransmitter::configCharacterMsg(40, 4, SCREEN_WIDTH / 2 - 150, 200, bulletsRemainingText, &textGraphic);

        PT_YIELD();
    }
    // Breaking out of the loop successfully calls this method,
    // allowing us to know that all execution is over.
    PT_END();
}


}  // namespace subsystems