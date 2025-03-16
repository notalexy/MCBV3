#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class LaneAssistLines : public GraphicsContainer {
public:
    LaneAssistLines() {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

private:
    const uint16_t BOTTOM_OFFSET = 480;
    const uint16_t TOP_OFFSET = 320;
    const uint16_t HEIGHT = 360;
    const uint16_t THICKNESS = 2;
    const uint16_t HALF_WIDTH = UISubsystem::SCREEN_WIDTH / 2;

    Line left{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(HALF_WIDTH - BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH - TOP_OFFSET), HEIGHT, THICKNESS};
    Line right{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(HALF_WIDTH + BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH + TOP_OFFSET), HEIGHT, THICKNESS};
};