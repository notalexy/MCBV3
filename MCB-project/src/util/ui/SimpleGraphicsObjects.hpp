#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/SimpleGraphicsObject.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class Line : public SimpleGraphicsObject {
public:
    Line(RefSerialData::Tx::GraphicColor graphicColor, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t thickness) : x1(x1), y1(y1), x2(x2), y2(y2), thickness(thickness) {
        color = graphicColor;
        UISubsystem::formatGraphicName(graphicNameArray, UISubsystem::getUnusedGraphicName());
    }

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) override {
        RefSerialTransmitter::configLine(thickness, x1, y1, x2, y2, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX1 == x1 && prevY1 == y1 && prevX2 == x2 && prevY2 == y2 && prevColor == color && hasDrawn); }

    uint16_t x1, y1, x2, y2, thickness;  // can set this directly, will appear next time drawn

protected:
    void setPrev() {
        prevThickness = thickness;
        prevX1 = x1;
        prevY1 = y1;
        prevX2 = x2;
        prevY2 = y2;
        prevColor = color;
    }

private:
    uint16_t prevThickness, prevX1, prevY1, prevX2, prevY2 = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class UnfilledRectangle : public Line {  // because unfilled rectangle and line store the exact same fields
public:
    UnfilledRectangle(RefSerialData::Tx::GraphicColor graphicColor, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t thickness) : Line(graphicColor, x1, y1, x2, y2, thickness) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) override {
        RefSerialTransmitter::configRectangle(thickness, x1, y1, x2, y2, graphicData);
        setPrev();
    }
};