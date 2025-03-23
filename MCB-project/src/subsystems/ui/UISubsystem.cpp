#include "UISubsystem.hpp"

namespace subsystems {

// define the static variable, if this isnt here things go wrong saying it is a undefined reference
uint32_t UISubsystem::currGraphicName = 0;

UISubsystem::UISubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers), drivers(drivers), refSerialTransmitter(drivers) {
    // the time it takes to send is dependent on the size of these structs, according to RefSerialData::Tx::getWaitTimeAfterGraphicSendMs()
    wasteIsBetterFor3 = sizeof(RefSerialData::Tx::Graphic1Message) + sizeof(RefSerialData::Tx::Graphic2Message) > sizeof(RefSerialData::Tx::Graphic5Message);
    wasteIsBetterFor4 = 2 * sizeof(RefSerialData::Tx::Graphic2Message) > sizeof(RefSerialData::Tx::Graphic5Message);
    wasteIsBetterFor6 = sizeof(RefSerialData::Tx::Graphic1Message) + sizeof(RefSerialData::Tx::Graphic5Message) > sizeof(RefSerialData::Tx::Graphic7Message);
    formatGraphicName(wasteNameArray, 0);
}

uint32_t UISubsystem::getUnusedGraphicName() {
    if (currGraphicName > 0xffffff) {
        // maybe signal some error? turn on the red led?
        return currGraphicName;
    } else {
        return currGraphicName++;
    }
}

uint8_t* UISubsystem::formatGraphicName(uint8_t array[3], uint32_t name) {
    // shifts and bitwise and
    array[0] = static_cast<uint8_t>((name >> 16) & 0xff);
    array[1] = static_cast<uint8_t>((name >> 8) & 0xff);
    array[2] = static_cast<uint8_t>(name & 0xff);
    return array;
}

void UISubsystem::initialize() { 
    this->restarting = true; 

    //temp for testing
    drivers->leds.init();
    drivers->leds.set(tap::gpio::Leds::Red, wasteIsBetterFor3);
    drivers->leds.set(tap::gpio::Leds::Green, wasteIsBetterFor4);
    drivers->leds.set(tap::gpio::Leds::Blue, wasteIsBetterFor6);
}

// guaranteed to be called, whether we have a command (topLevelContainer) or not
void UISubsystem::refresh() { 
    if(topLevelContainer){
        if(!startTime) startTime = tap::arch::clock::getTimeMicroseconds();
        topLevelContainer->update();
        run(); 
    }
}

bool UISubsystem::run() {
    // The thread has exited the loop, meaning that there are no locked resources
    if (!this->isRunning()) {
        restart();// Restart the thread

        // Reset the HUD elements
        // don't reset currGraphicName because old objects don't lose their names they just need to re-add themselves
        restarting = false;
        needToDelete = true;
    }

    PT_BEGIN();
    // inside of a protothread, you aren't able to make new variables

    PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

    // need to figure out delete at start. It seems like it wants to delete every time run is called
    if(needToDelete){
        needToDelete = false; //probably this needs to be first to not go in this branch next time, so set it to false before the pt call
    //  PT_CALL(refSerialTransmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_ALL, 0));
    //  topLevelContainer->hasBeenCleared();
        timesDeleted++;
    }

    // drivers->leds.set(tap::gpio::Leds::Blue, timesDeleted==0);
    // drivers->leds.set(tap::gpio::Leds::Green, timesDeleted==1);
    // drivers->leds.set(tap::gpio::Leds::Red, timesDeleted>1);

    // If we try to restart the hud, break out of the loop
    while (!this->restarting) {
        graphicsIndex = 0;                                  
        nextGraphicsObject = topLevelContainer->getNext();  // if nullptr on first call, the first while loop is skipped
        while (nextGraphicsObject) {
            if(nextGraphicsObject->isStringGraphic()){
                //if it is a string, keep the array as it is and send the string on its own
                nextGraphicsObject->configCharacterData(&messageCharacter);
                PT_CALL(refSerialTransmitter.sendGraphic(&messageCharacter));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&messageCharacter));
                PT_WAIT_UNTIL(delayTimeout.execute());
            } else {
                //if it isn't a string, add it to the array and see if it is full
                objectsToSend[graphicsIndex++] = nextGraphicsObject;
                if (graphicsIndex == TARGET_NUM_OBJECTS) break;
            }
            nextGraphicsObject = topLevelContainer->getNext();
        }
        // after the loop we know calling again getNext will return nullptr or we have filled with all seven objects
        if (graphicsIndex != 7) {
            // so calling getNext would return nullptr, lets try reseting the iteration
            topLevelContainer->resetIteration();
            updateFPS();
            nextGraphicsObject = topLevelContainer->getNext();  // if nullptr again after reset iteration, the second while loop is skipped
            //same while loop as above, would make into a function except for the PT_CALL, would need to be a resumable function and that makes it complicated
            //maybe make it so it is a while loop inside of a while loop
            while (nextGraphicsObject) {
                if(nextGraphicsObject->isStringGraphic()){
                    PT_CALL(refSerialTransmitter.sendGraphic(&messageCharacter));
                    delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&messageCharacter));
                    PT_WAIT_UNTIL(delayTimeout.execute());
                } else {
                    objectsToSend[graphicsIndex++] = nextGraphicsObject;
                    if (graphicsIndex == TARGET_NUM_OBJECTS) break;
                }
                nextGraphicsObject = topLevelContainer->getNext();
            }
        }

        // so we have up to 7 objects to update. Would do a switch case but might confict with protothread's switch case
        if (graphicsIndex == 1) {
            objectsToSend[0]->configGraphicData(&message1.graphicData);
            PT_CALL(refSerialTransmitter.sendGraphic(&message1));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message1));
        } else if (graphicsIndex == 2) {
            for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) 
                objectsToSend[innerGraphicsIndex]->configGraphicData(&message2.graphicData[innerGraphicsIndex]);
            PT_CALL(refSerialTransmitter.sendGraphic(&message2));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message2));
        } else if (graphicsIndex == 5) {
            for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++)
                 objectsToSend[innerGraphicsIndex]->configGraphicData(&message5.graphicData[innerGraphicsIndex]);
            PT_CALL(refSerialTransmitter.sendGraphic(&message5));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message5));
        } else if (graphicsIndex == 7) {
            for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) 
                objectsToSend[innerGraphicsIndex]->configGraphicData(&message7.graphicData[innerGraphicsIndex]);
            PT_CALL(refSerialTransmitter.sendGraphic(&message7));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message7));
        } else if (graphicsIndex == 3) {
            // not sure if we would rather send a 5 with 2 wasted slots or a 2 and a 1
            // good thing there is a variable for it
            if (wasteIsBetterFor3) {
                // send 5 with wasted space (no op commands)
                // fill 3
                for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) 
                    objectsToSend[innerGraphicsIndex]->configGraphicData(&message5.graphicData[innerGraphicsIndex]);

                // waste 2
                for (; innerGraphicsIndex < 5; innerGraphicsIndex++) {
                    RefSerialTransmitter::configGraphicGenerics(
                        &message5.graphicData[innerGraphicsIndex],
                        wasteNameArray,
                        RefSerialData::Tx::GraphicOperation::GRAPHIC_NO_OP,
                        0,
                        RefSerialData::Tx::GraphicColor::RED_AND_BLUE);
                    RefSerialTransmitter::configLine(0, 0, 0, 0, 0, &message5.graphicData[innerGraphicsIndex]);  // not sure if this is necessary since we say no-op, but just in case
                }
                
                PT_CALL(refSerialTransmitter.sendGraphic(&message5));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message5));
            } else {
                // send 2
                for (innerGraphicsIndex = 0; innerGraphicsIndex < 2; innerGraphicsIndex++) 
                    objectsToSend[innerGraphicsIndex]->configGraphicData(&message2.graphicData[innerGraphicsIndex]);
                PT_CALL(refSerialTransmitter.sendGraphic(&message2));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message2));
                PT_WAIT_UNTIL(delayTimeout.execute());  // need to wait for the first, the second one gets waited on after

                // send 1
                objectsToSend[2]->configGraphicData(&message1.graphicData);
                PT_CALL(refSerialTransmitter.sendGraphic(&message1));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message1));
            }
        } else if (graphicsIndex == 4) {
            if (wasteIsBetterFor4) {
                // send 5 with wasted space (no op commands)
                // fill 4
                for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) 
                    objectsToSend[innerGraphicsIndex]->configGraphicData(&message5.graphicData[innerGraphicsIndex]);

                // waste 1
                RefSerialTransmitter::configGraphicGenerics(
                    &message5.graphicData[4],
                    wasteNameArray,
                    RefSerialData::Tx::GraphicOperation::GRAPHIC_NO_OP,
                    0,
                    RefSerialData::Tx::GraphicColor::RED_AND_BLUE);
                RefSerialTransmitter::configLine(0, 0, 0, 0, 0, &message5.graphicData[4]);  // not sure if this is necessary since we say no-op, but just in case

                PT_CALL(refSerialTransmitter.sendGraphic(&message5));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message5));
            } else {
                // send 2
                for (innerGraphicsIndex = 0; innerGraphicsIndex < 2; innerGraphicsIndex++) 
                    objectsToSend[innerGraphicsIndex]->configGraphicData(&message2.graphicData[innerGraphicsIndex]);
                PT_CALL(refSerialTransmitter.sendGraphic(&message2));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message2));
                PT_WAIT_UNTIL(delayTimeout.execute());  // need to wait for the first, the second one gets waited on after

                // send 2
                for (innerGraphicsIndex = 0; innerGraphicsIndex < 2; innerGraphicsIndex++)
                    objectsToSend[innerGraphicsIndex + 2]->configGraphicData(&message2.graphicData[innerGraphicsIndex]);
                PT_CALL(refSerialTransmitter.sendGraphic(&message2));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message2));
            }
        } else if (graphicsIndex == 6) {
            if (wasteIsBetterFor6) {
                // send 7 with wasted space (no op commands)
                // fill 6
                for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) 
                    objectsToSend[innerGraphicsIndex]->configGraphicData(&message7.graphicData[innerGraphicsIndex]);

                // waste 1
                RefSerialTransmitter::configGraphicGenerics(
                    &message7.graphicData[6],
                    wasteNameArray,
                    RefSerialData::Tx::GraphicOperation::GRAPHIC_NO_OP,
                    0,
                    RefSerialData::Tx::GraphicColor::RED_AND_BLUE);
                RefSerialTransmitter::configLine(0, 0, 0, 0, 0, &message7.graphicData[6]);  // not sure if this is necessary since we say no-op, but just in case

                PT_CALL(refSerialTransmitter.sendGraphic(&message7));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message7));
            } else {
                // send 5
                for (innerGraphicsIndex = 0; innerGraphicsIndex < 5; innerGraphicsIndex++) 
                    objectsToSend[innerGraphicsIndex]->configGraphicData(&message5.graphicData[innerGraphicsIndex]);
                PT_CALL(refSerialTransmitter.sendGraphic(&message5));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message5));
                PT_WAIT_UNTIL(delayTimeout.execute());  // need to wait for the first, the second one gets waited on after

                // send 1
                objectsToSend[5]->configGraphicData(&message1.graphicData);
                PT_CALL(refSerialTransmitter.sendGraphic(&message1));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message1));
            }
        }
        

        PT_WAIT_UNTIL(delayTimeout.execute());

        PT_YIELD();
    }
    // Breaking out of the loop successfully calls this method,
    // allowing us to know that all execution is over.
    PT_END();
}

void UISubsystem::updateFPS() {
    //the timer wraps every 72 minutes, when it does just don't update fps that frame
    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    if((currentTime - startTime)>0)
        fps = 1e6f / (currentTime - startTime);
    startTime = currentTime;
}

//This is required for the UISubsystem to have anything to draw.
void UISubsystem::setTopLevelContainer(GraphicsContainer* container) {
    topLevelContainer = container;
}

}  // namespace subsystems