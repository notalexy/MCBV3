#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

// looks like / \ at the bottom of the screen
class LaneAssistLines : public GraphicsContainer {
public:
    LaneAssistLines(GimbalSubsystem* gimbal) : gimbal (gimbal) {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    void update() final {
        //these lines could change with the gimbal looking up and down
        //but that is currently unimplemented
    }

private:
    static constexpr uint16_t BOTTOM_OFFSET = 480; //distance from the center of the screen to the bottom of each line
    static constexpr uint16_t TOP_OFFSET = 320; //distance from the center of the screen to the top of each line, should be less than BOTTOM_OFFSET
    static constexpr uint16_t HEIGHT = 360; //distance from bottom of the screen to the top of each line
    static constexpr uint16_t THICKNESS = 2; //pixels

    GimbalSubsystem* gimbal;

    Line left{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - BOTTOM_OFFSET), 0, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - TOP_OFFSET), HEIGHT, THICKNESS};
    Line right{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH + BOTTOM_OFFSET), 0, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH + TOP_OFFSET), HEIGHT, THICKNESS};
};