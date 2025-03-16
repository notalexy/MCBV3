#pragma once

#include "util/ui/GraphicsObject.hpp"

class SimpleGraphicsObject : public GraphicsObject {
public:
    int countNeedRedrawn() final { return needsRedrawn(); }

    /*
     * Inheriting simple objects should keep track of what they drew
     * previously with, and compare that to what they want to be drawn with
     * */
    virtual bool needsRedrawn() = 0;

    GraphicsObject* getNext() final {
        if (countIndex == 0 && needsRedrawn()) {  // could do !countIndex
            countIndex = 1;
            return this;
        }
        return nullptr;
    }

    int size() final {
        return 1;  // container of one object
    }

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) = 0;

    void configGraphicData(RefSerialData::Tx::GraphicData* graphicData) final  {
        RefSerialTransmitter::configGraphicGenerics(
            graphicData,
            graphicNameArray,
            hasDrawn ? RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY : RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD,
            0,
            color);
        hasDrawn = true;
        finishConfigGraphicData(graphicData);
    }
    
    
    RefSerialData::Tx::GraphicColor color; //can set this directly, will appear next time drawn

protected:

    bool hasDrawn = false; //to determine if we need to create or update
    
    uint8_t graphicNameArray[3];
};